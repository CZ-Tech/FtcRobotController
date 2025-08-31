package org.firstinspires.ftc.teamcode.common.drive;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;

import java.util.Arrays;

/**
 * 一个运动控制器，它使用地面摩擦力的自适应模型来规划点运动的最快加速和减速曲线。
 * 它与防抱死牵引系统（ATS）协作，以计算加速度和防止车轮打滑。
 *
 * 核心功能:
 * 1.  自适应学习 (Adaptive Learning): 根据ATS的反馈动态估算最大可实现加速度，以适应不同的场地表面。
 * 2.  运动规划 (Motion Profiling): 计算到达目标的速度曲线，包括加速、巡航和刹车阶段，以确保精确停止或通过。
 * 4.  两段式控制 (Two-Stage Control): 在远距离时使用平移+旋转的复合运动，在需要停止时切换到纯旋转以实现高精度的最终对准。
 * 5.  牵引力控制 (Traction Control): 将最终的动力施加任务交给ATS，以保证在所有条件下获得最大抓地力并防止打滑。
 */
public class AdaptiveTractionController {
    private final GoBildaPinpointDataAsync odo;
    private final AntiLockTractionSystem ATS;

    private double[] targetPos = new double[3];
    private double[] speedOnTargetPos = new double[3];

    private Modes mode = Modes.RUN_ANTILOCK_ONLY;

    // Control constants - Adjust these values as needed

    // 用于判断运动是否“足够直”的阈值 (0 到 1 之间)
    // 1.0 表示只在完美直线上学习，0.0 表示在任何运动中都学习。
    // 一个较高的值 (如 0.9) 能有效过滤掉曲线运动数据，同时允许近似直线运动的数据被采纳。
    public static double STRAIGHTNESS_THRESHOLD = 0.9;
    // 这是一个比值，填的越大，允许转向调度的功率越多。1表示最大调度三分之一的功率。但是无论调多大都不会在所有情况下完全给旋转分配功率
    public static final double MAX_TURN_POWER = 1;
    public static final double HEADING_P_GAIN_FAR = 0.5; // 偏离目标较远时航向的比例增益（每弧度误差的弧度/秒）
    public static final double HEADING_P_GAIN_CLOSE = 1.0; // 接近目标时航向的比例增益（以弧度每弧度误差的单位表示，单位为弧度/秒）
    // 视为抵达的距离阈值
    public static final double STOP_THRESHOLD_MM = 20.0;
    // 强制提前刹车的距离(mm)，防止实际摩擦力较小导致过冲
    public static final double BRAKE_LEAD_MM = 50.0;

    // --- 自适应学习与运动规划常量 ---
    // 机器人的最大巡航速度 (mm/s)，这是一个安全上限
    public static double MAX_CRUISE_SPEED_MM_S = 1500.0;
    // 对最大加速度的初始猜测值 (mm/s^2)，一个保守的启动值
    public static double INITIAL_MAX_ACCEL_MM_S2 = 2000.0;
    // 加速度学习率，用于指数移动平均滤波器 (0到1之间)
    // 较小的值意味着学习更平滑但更慢
    public static double ACCEL_LEARNING_RATE = 0.05;

    private static final int MIN_MOTORS_LIMITED_FOR_LEARNING = 3;  // 认为达到最大加速度时ATS介入的最小电机数

    public static double MAX_YAW_SPEED_WHEN_LEARNING = 5;  // 在触发学习加速度时允许的最大转向角速度 (rad/s)

    // --- 主动刹车常量 ---
    // 视为已停止的速度阈值 (mm/s)
    public static final double BRAKE_STOP_VELOCITY_THRESHOLD_MM_S = 20.0;
    // 视为已停止的角速度阈值 (rad/s)
    public static final double BRAKE_STOP_YAW_RATE_THRESHOLD_RAD_S = 0.05;
    // 用于抑制旋转的P增益
    public static final double BRAKE_YAW_P_GAIN = 0.8;

    private Runnable reachCallback;

    // --- 状态变量 ---
    // 动态学习到的、机器人在此地面上能达到的最大加速度
    private double learnedMaxAcceleration = INITIAL_MAX_ACCEL_MM_S2;
    // 用于计算加速度的时间和速度变量
    private long lastUpdateTimeNanos = 0;
    private double lastVelX = 0;
    private double lastVelY = 0;

    public double getLearnedMaxAcceleration() {
        return learnedMaxAcceleration;
    }

    public static enum Modes {
        RUN_TO_POSITION,
        RUN_ANTILOCK_ONLY,
        AUTO_BRAKE
    }


    public AdaptiveTractionController(GoBildaPinpointDataAsync odo, AntiLockTractionSystem ats) {
        this.odo = odo;
        this.ATS = ats;
        resetLearning();
    }

    /**
     * 将学习到的加速度值重置为初始状态。
     * 应在每次自动程序开始时调用。
     */
    public void resetLearning() {
        this.learnedMaxAcceleration = INITIAL_MAX_ACCEL_MM_S2;
        this.lastUpdateTimeNanos = 0;
        this.lastVelX = 0;
        this.lastVelY = 0;
        Arrays.fill(speedOnTargetPos, 0.0);
    }

    /**
     * 自适应学习模块：根据ATS反馈更新最大加速度
     * (最终版本 - 结合了精确的矢量加速度计算和智能的直线度过滤)
     */
    private void updateFrictionLearning() {
        if (lastUpdateTimeNanos == 0) {
            lastUpdateTimeNanos = System.nanoTime();
            // TODO 这里的X、Y顺序可能有误
            lastVelX = odo.getVelX();
            lastVelY = odo.getVelY();
            return;
        }

        long currentTimeNanos = System.nanoTime();
        double dt = (currentTimeNanos - lastUpdateTimeNanos) / 1e9;
        if (dt < 1e-6) return;

        // --- 1. 计算精确的加速度矢量 ---
        // TODO 这里的X、Y顺序可能有误
        double currentVelX = odo.getVelX();
        double currentVelY = odo.getVelY();

        double accelX = (currentVelX - lastVelX) / dt;
        double accelY = (currentVelY - lastVelY) / dt;

        double currentAccelerationMagnitude = Math.hypot(accelX, accelY);

        // --- 2. 检查学习前提条件 ---
        // a) 牵引力是否达到极限 (由ATS判断)
        // b) 机器人自身是否在快速旋转 (由yawRate判断)
        double yawRate = odo.getHeadingVelocity();
        if (ATS.isUsingLimitingFriction(MIN_MOTORS_LIMITED_FOR_LEARNING) && Math.abs(yawRate) < MAX_YAW_SPEED_WHEN_LEARNING) {

            // --- 3. 智能过滤：判断运动是否“足够直” ---
            double currentVelocityMagnitude = Math.hypot(currentVelX, currentVelY);

            // 为了避免除以零，仅在机器人有速度和加速度时才计算直线度
            if (currentVelocityMagnitude > 1e-6 && currentAccelerationMagnitude > 1e-6) {

                // 计算速度矢量v和加速度矢量a的点积
                double dotProduct = currentVelX * accelX + currentVelY * accelY;

                // 计算 |cos(θ)| = |(v · a) / (|v| * |a|)|
                double straightnessFactor = Math.abs(dotProduct / (currentVelocityMagnitude * currentAccelerationMagnitude));

                // 只有当运动足够直时，才进行学习
                if (straightnessFactor > STRAIGHTNESS_THRESHOLD) {
                    // 使用指数移动平均 (EMA) 来平滑地更新学习到的值
                    learnedMaxAcceleration = (1 - ACCEL_LEARNING_RATE) * learnedMaxAcceleration
                            + ACCEL_LEARNING_RATE * currentAccelerationMagnitude;
                }
            }
        }

        // --- 4. 为下一次迭代更新状态 ---
        lastUpdateTimeNanos = currentTimeNanos;
        lastVelX = currentVelX;
        lastVelY = currentVelY;
    }

    public AdaptiveTractionController(GoBildaPinpointDataAsync odo) {
        this.odo = odo;
        AntiLockTractionSystem.ATSConfig config = AntiLockTractionSystem.ATSConfig.getBuilder()
                .setAntislipPowerChangeFactor(Globals.ANTISLIP_POWER_CHANGE_FACTOR)
                .setAtsLockThreshold(Globals.ATS_LOCK_THRESHOLD)
                .setAtsPowerChangeStep(Globals.ATS_POWER_CHANGE_STEP)
                .setAtsStartPower(Globals.ATS_START_POWER)
                .setLateralDistance(Globals.LATERAL_DISTANCE)
                .setWheelbase(Globals.WHEELBASE)
                .setCountsPerMotorRev(Globals.COUNTS_PER_MOTOR_REV)
                .setWheelDiameterInches(Globals.WHEEL_DIAMETER_INCHES)
                .build();
        ATS = new AntiLockTractionSystem(odo, config);
    }

    /**
     * 该方法也不会主动更新odo数据！！！
     * 这是为异步更新架构准备的
     */
    public void update() {
        // 先学习加速度
        updateFrictionLearning();
        // 再执行核心逻辑
        switch (mode) {
            case AUTO_BRAKE:
                runAutoBrake();
                break;
            case RUN_ANTILOCK_ONLY:
                break;
            case RUN_TO_POSITION:
                double distanceToTarget = runToPosition();
                // 先检测是否达到目标
                if (distanceToTarget <= STOP_THRESHOLD_MM && reachCallback != null) {
                    // 达到则执行回调
                    reachCallback.run();
                }
                break;
        }
        // 更新 ATS 系统以启用需协作的功能并处理防抱死系统相关问题。
        ATS.update();
    }

    /**
     * 自动规划行驶到目标点
     * @return 到目标的剩余距离
     */
    public double runToPosition() {
        // 获取机器人当前在世界坐标系下的实时位姿
        double currentX = odo.getPosX();
        double currentY = odo.getPosY();
        double currentHeading = odo.getHeading();

        // 计算从当前位置指向目标位置的世界坐标系下的误差向量 (dx_world, dy_world)
        double dx_world = targetPos[0] - currentX;
        double dy_world = targetPos[1] - currentY;
        // 计算当前位置到目标位置的直线距离（标量，永远为正）
        double distanceToTarget = Math.hypot(dx_world, dy_world);

        // 初始化动力输出变量
        double axialPower = 0.0, lateralPower = 0.0, yawPower = 0.0;

        // 采用两段式控制策略, 在接近终点停止时，通过切换到纯旋转来保证停靠的最终精度和稳定性。
        if (!isTargetSpeedEffectivelyZero() || distanceToTarget > STOP_THRESHOLD_MM) {
            // --- 复合运动阶段 (平移 + 旋转) ---

            // ==================== 刹车距离计算 ====================
            // 获取当前世界坐标系下的速度
            // TODO 这里的X、Y顺序可能有误
            double currentVelX = odo.getVelX();
            double currentVelY = odo.getVelY();

            // 将速度矢量(velX, velY)投影到位置误差矢量(dx_world, dy_world)上。
            // 得到的结果是机器人“朝向”目标点的速度分量，这才是决定何时刹车的关键。
            // 公式: v_proj = (velocity · distance_vec) / |distance_vec|
            double speedTowardsTarget = 0.0;
            if (distanceToTarget > 1e-6) { // 防止除以零
                speedTowardsTarget = (currentVelX * dx_world + currentVelY * dy_world) / distanceToTarget;
            }

            // 如果机器人正在远离目标（投影为负），我们不考虑刹车。只在接近时才计算。
            // 远离时应该全力加速以返回，而不是刹车，故就放个0避免后续平方出正的计算出错误刹车距离
            speedTowardsTarget = Math.max(0, speedTowardsTarget);

            // 这是旧的基于目标速度为0的刹车点算法 -> // 使用修正后的、更准确的速度分量来计算刹车距离
            // double brakingDistance = (speedTowardsTarget * speedTowardsTarget) / Math.max(2 * learnedMaxAcceleration, 1e-6);

            // 下面是新的
            // 计算目标速度在接近方向上的投影分量，作为减速的终点速度
            double targetSpeedProjected = 0.0;
            if (distanceToTarget > 1e-6) {
                // 使用点积计算
                // 这里的投影速度可以是负数，代表希望在目标点时是背离目标方向运动的（例如过点反向）。
                // targetSpeedProjected 表示目标速度在目标方向上的分量大小。如果值为正，表示到目标后继续向前；如果值为负，表示到目标后立刻倒车。
                targetSpeedProjected = (speedOnTargetPos[0] * dx_world + speedOnTargetPos[1] * dy_world) / distanceToTarget;
            }

            // 使用通用的运动学公式 d = (v_initial² - v_final²) / (2a) 计算减速所需的距离
            double brakingDistance = 0.0;
            // 只有在当前速度的平方大于目标速度的平方时，才需要减速。
            // 这个公式天然地处理了 targetSpeedProjected 为正、零的情况。若targetSpeedProjected为负，在实际情况下它应该会尽快停住
            double speedDiffSq = speedTowardsTarget * speedTowardsTarget - targetSpeedProjected * targetSpeedProjected;
            if (speedDiffSq > 0) {
                brakingDistance = speedDiffSq / Math.max(2 * Math.abs(learnedMaxAcceleration), 1e-6);
            }

            // ==========================================================

            // 将世界坐标系下的误差向量，通过旋转矩阵投影到机器人自身的坐标系上。
            double cos_h = Math.cos(currentHeading);
            double sin_h = Math.sin(currentHeading);
            double dx_robot = dx_world * cos_h + dy_world * sin_h;
            double dy_robot = -dx_world * sin_h + dy_world * cos_h;

            // 定义“行进”的符号
            // 目标在前方，基准为前进 (+1)；在后方，基准为后退 (-1)。
            double directionSign = (dx_robot >= 0) ? 1.0 : -1.0;

//            // （旧算法）计算刹车距离
//            double brakingDistance = (lastVelocityMagnitude * lastVelocityMagnitude) / Math.max(2 * learnedMaxAcceleration, 1e-6);
            // 定义油门方向的符号
            // 在刹车区外则+1；在刹车区内则-1。（distanceToTarget始终为正）
            double throttleSign = (distanceToTarget > brakingDistance + BRAKE_LEAD_MM) ? 1.0 : -1.0;

            // 最终输出动力符号 = 方向策略 * 油门方向
            double targetPowerSign = directionSign * throttleSign;

            // 计算机器人应该施加推力的方向（在机器人坐标系下）。
            // - 当前进时(directionSign=+1), moveAngle = atan2(dy_robot, dx_robot)，即目标方向。
            // - 当后退时(directionSign=-1), moveAngle = atan2(-dy_robot, -dx_robot)，即目标的相反方向。
            double moveAngle = Math.atan2(directionSign * dy_robot, directionSign * dx_robot);

            // 8. 计算最终动力分量
            axialPower = targetPowerSign * Math.cos(moveAngle);
            lateralPower = targetPowerSign * Math.sin(moveAngle);

            // 转向逻辑与平移逻辑解耦，它只负责在整个移动过程中，持续地将机器人朝向最终的目标姿态。
            double headingError = normalizeAngle(targetPos[2] - currentHeading);
            double rawFarYawPower = headingError * HEADING_P_GAIN_FAR;
            yawPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawFarYawPower));

        } else {
            // --- 阶段2: 近距离纯旋转阶段 ---
            // 当机器人足够接近目标点时，停止所有平移动作。
            axialPower = 0.0;
            lateralPower = 0.0;
            // 此时只执行纯旋转，以更高的P增益进行精确的姿态对准。
            double headingError = normalizeAngle(targetPos[2] - currentHeading);
            double rawCloseYawPower = headingError * HEADING_P_GAIN_CLOSE;
            yawPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawCloseYawPower));
        }

        ATS.setTargetPower(axialPower, lateralPower, yawPower);
        return distanceToTarget;
    }

    /**
     * 辅助函数，用于检查目标速度矢量是否有效为零，以处理浮点数精度问题。
     */
    private boolean isTargetSpeedEffectivelyZero() {
        final double epsilon = 1e-6; // 定义一个极小值作为阈值
        return Math.abs(speedOnTargetPos[0]) < epsilon &&
                Math.abs(speedOnTargetPos[1]) < epsilon &&
                Math.abs(speedOnTargetPos[2]) < epsilon;
    }

    // 用于将角度归一化至 [-π, π] 范围内的辅助方法
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * 执行主动刹车的核心逻辑。
     * 该方法计算与当前运动相反的动力矢量，并将其交给ATS执行。
     */
    private void runAutoBrake() {
        // 1. 获取机器人在世界坐标系下的当前速度
        double currentVelX_world = odo.getVelX();
        double currentVelY_world = odo.getVelY();
        double currentYawRate = odo.getHeadingVelocity(); // 角速度

        double translationalSpeed = Math.hypot(currentVelX_world, currentVelY_world);

        // 2. 检查是否已经停止
        if (translationalSpeed < BRAKE_STOP_VELOCITY_THRESHOLD_MM_S &&
                Math.abs(currentYawRate) < BRAKE_STOP_YAW_RATE_THRESHOLD_RAD_S) {
            // 已经完全停止，不再施加动力，并可以切换回默认模式
            ATS.setTargetPower(0, 0, 0);
            mode = Modes.RUN_ANTILOCK_ONLY; // 刹车完成，恢复到空闲状态
            return;
        }

        // 3. 将世界速度矢量转换为机器人本地坐标系
        double currentHeading = odo.getHeading();
        double cos_h = Math.cos(currentHeading);
        double sin_h = Math.sin(currentHeading);

        // 这是机器人前进/后退方向上的速度分量
        double vel_robot_axial = currentVelX_world * cos_h + currentVelY_world * sin_h;
        // 这是机器人向左/向右方向上的速度分量
        double vel_robot_lateral = -currentVelX_world * sin_h + currentVelY_world * cos_h;

        // 4. 计算与运动方向完全相反的动力矢量
        // 我们想要一个与 (vel_robot_axial, vel_robot_lateral) 方向相反的单位矢量
        double axialPower = -vel_robot_axial;
        double lateralPower = -vel_robot_lateral;

        // 归一化平移动力矢量，使其模长为1，只保留方向信息
        // 这样ATS就能以最大功率进行刹车
        if (translationalSpeed > 1e-6) { // 防止除以零
            axialPower /= translationalSpeed;
            lateralPower /= translationalSpeed;
        }

        // 5. 计算抑制旋转的动力
        // 使用一个简单的P控制器来减小角速度
        double yawPower = -currentYawRate * BRAKE_YAW_P_GAIN;

        // 6. 将最终的刹车动力指令发送给ATS
        ATS.setTargetPower(axialPower, lateralPower, yawPower);
    }

    /**
     * 启动主动刹车模式。
     * 在此模式下，控制器将持续施加与机器人当前运动相反的动力，
     * 直到机器人完全停止（平移和旋转速度均低于阈值）。
     * 调用此方法后，您必须继续在循环中调用 update() 来执行刹车过程。
     */
    public void autoBrake() {
        this.mode = Modes.AUTO_BRAKE;
    }

    public void setTargetPos(double x, double y, double heading) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = heading;
        reachCallback = () -> Robot.getInstance().telemetry.addLine("Reached target position");
    }

    public void setTargetPos(double x, double y, double heading, Runnable reachCallback) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = heading;
        this.reachCallback = reachCallback;
    }

    public void setTargetPos(double x, double y) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = odo.getHeading(); // Maintain current heading for this overload
        reachCallback = () -> Robot.getInstance().telemetry.addLine("Reached target position");
    }

    public void setTargetPos(double x, double y, Runnable reachCallback) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = odo.getHeading(); // Maintain current heading for this overload
        this.reachCallback = reachCallback;
    }

    public void setReachCallback(Runnable reachCallback) {
        this.reachCallback = reachCallback;
    }

    /**
     * 用于设定路过目标时的速度
     * TODO！！！注意：程序只保证达成目标方向上的速度分量！！！
     * @param x x方向速度
     * @param y y方向速度
     * @param yaw 旋转速度（跟没有一样）
     */
    @Deprecated
    public void setSpeedOnTargetPos(double x, double y, double yaw) {
        speedOnTargetPos[0] = x;
        speedOnTargetPos[1] = y;
        speedOnTargetPos[2] = yaw;
    }

    /**
     * 用于设定路过目标时的速度
     * TODO！！！注意：程序只尽力达成目标方向上的速度分量！！！
     * @param x x方向速度
     * @param y y方向速度
     */
    public void setSpeedOnTargetPos(double x, double y) {
        speedOnTargetPos[0] = x;
        speedOnTargetPos[1] = y;
        speedOnTargetPos[2] = 0;
    }

    public void setRunToPos() {
        mode = Modes.RUN_TO_POSITION;
    }

    public void setRunAntiLock() {
        mode = Modes.RUN_ANTILOCK_ONLY;
    }

    public Modes getMode() {
        return mode;
    }

}