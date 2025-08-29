package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.drive.AntiLockTractionSystem;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;

import java.util.Arrays;

public class AdaptiveTractionController {
    private GoBildaPinpointDataAsync odo;
    private AntiLockTractionSystem ATS;

    private double[] targetPos = new double[3];
    private double[] speedOnTargetPos = new double[3];

    private Modes mode = Modes.RUN_ANTILOCK_ONLY;

    // Control constants - Adjust these values as needed
    public static final double MAX_TURN_POWER = 0.5;
    public static final double HEADING_P_GAIN_FAR = 0.5; // 偏离目标较远时航向的比例增益（每弧度误差的弧度/秒）
    public static final double HEADING_P_GAIN_CLOSE = 1.0; // 接近目标时航向的比例增益（以弧度每弧度误差的单位表示，单位为弧度/秒）
    // STOP_THRESHOLD_MM 现在定义了从复合运动切换到纯旋转的距离
    public static final double STOP_THRESHOLD_MM = 20.0;

    // --- 新增：自适应学习与运动规划常量 ---
    // 机器人的最大巡航速度 (mm/s)，这是一个安全上限
    public static double MAX_CRUISE_SPEED_MM_S = 1500.0;
    // 对最大加速度的初始猜测值 (mm/s^2)，一个保守的启动值
    public static double INITIAL_MAX_ACCEL_MM_S2 = 2000.0;
    // 加速度学习率，用于指数移动平均滤波器 (0到1之间)
    // 较小的值意味着学习更平滑但更慢
    public static double ACCEL_LEARNING_RATE = 0.05;

    public static double MAX_YAW_SPEED_WHEN_LEARNING = 5;  // 在触发学习加速度时允许的最大转向角速度 (rad/s)

    // --- 新增：状态变量 ---
    // 动态学习到的、机器人在此地面上能达到的最大加速度
    private double learnedMaxAcceleration = INITIAL_MAX_ACCEL_MM_S2;
    // 用于计算加速度的时间和速度变量
    private long lastUpdateTimeNanos = 0;
    private double lastVelocityMagnitude = 0;

    public static enum Modes {
        RUN_TO_POSITION,
        RUN_ANTILOCK_ONLY
    }

    public enum state {
        MIXED,
        BREAKING,
        ACCELERATING,  // 直线加速
        TURNING,
        CONSTANT,  // 匀速
        STOPPED
    }

    public AdaptiveTractionController(GoBildaPinpointDataAsync odo, AntiLockTractionSystem ats) {
        this.odo = odo;
        this.ATS = ats;
        resetLearning();
    }

    /**
     * 重置学习到的加速度值，在每次自动程序开始时调用
     */
    public void resetLearning() {
        this.learnedMaxAcceleration = INITIAL_MAX_ACCEL_MM_S2;
        this.lastUpdateTimeNanos = 0;
        this.lastVelocityMagnitude = 0;
    }

    /**
     * 自适应学习模块：根据ATS反馈更新最大加速度
     */
    private void updateFrictionLearning() {
        if (lastUpdateTimeNanos == 0) {
            lastUpdateTimeNanos = System.nanoTime();
            return;
        }

        long currentTimeNanos = System.nanoTime();
        double dt = (currentTimeNanos - lastUpdateTimeNanos) / 1e9; // 转换为秒
        if (dt < 1e-6) return; // 避免除零

        double currentVelX = odo.getVelX();
        double currentVelY = odo.getVelY();
        double currentVelocityMagnitude = Math.hypot(currentVelX, currentVelY);

        // 计算当前加速度的瞬时值
        double currentAcceleration = Math.abs(currentVelocityMagnitude - lastVelocityMagnitude) / dt;

        // 如果ATS正在介入（意味着我们达到了摩擦力极限），就更新我们的模型
        double yawRate = odo.getHeadingVelocity(); // 假设 odo 提供角速度
        if (ATS.isUsingLimitingFriction() && Math.abs(yawRate) < MAX_YAW_SPEED_WHEN_LEARNING) {
            // 使用指数移动平均 (EMA) 来平滑地更新学习到的值
            learnedMaxAcceleration = (1 - ACCEL_LEARNING_RATE) * learnedMaxAcceleration
                    + ACCEL_LEARNING_RATE * currentAcceleration;
        }

        // 为下一次迭代更新状态
        lastUpdateTimeNanos = currentTimeNanos;
        lastVelocityMagnitude = currentVelocityMagnitude;
    }

    private AdaptiveTractionController(GoBildaPinpointDataAsync odo) {
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
        updateFrictionLearning();
        switch (mode) {
            case RUN_TO_POSITION:
                double currentX = odo.getPosX();
                double currentY = odo.getPosY();
                double currentHeading = odo.getHeading(); // 单位:弧度(radians)

                double dx = targetPos[0] - currentX;
                double dy = targetPos[1] - currentY;
                double distanceToTarget = Math.hypot(dx, dy);

                // 获取当前朝向与目标的差值
                double headingError = normalizeAngle(targetPos[2] - currentHeading);

                double axialPower = 0.0;
                double lateralPower = 0.0;
                double yawPower = 0.0;
                if (distanceToTarget > STOP_THRESHOLD_MM) {
                    // 阶段1：复合运动（平移+旋转）
                    // 计算最短刹车距离(朴实无华的 v^2 = 2ax)
                    double brakingDistance = (lastVelocityMagnitude * lastVelocityMagnitude) / (2 * learnedMaxAcceleration);
                    double targetSpeed;
                    if (distanceToTarget <= brakingDistance) {
                        // **进入刹车阶段**
                        // 我们需要减速以在终点停下。目标速度是基于剩余距离的函数。
                        // v_target = sqrt(2 * a * d)
                        targetSpeed = Math.sqrt(2 * learnedMaxAcceleration * distanceToTarget);
                    } else {
                        // **加速/巡航阶段**
                        // 我们可以安全地加速到最大速度
                        targetSpeed = MAX_CRUISE_SPEED_MM_S;
                    }

                    // 将目标速度转换为0-1范围的功率值
                    double translationalPower = Math.min(targetSpeed / MAX_CRUISE_SPEED_MM_S, 1.0);

                    // 计算移动方向
                    double angleToTarget = Math.atan2(dy, dx);
                    double moveAngle = normalizeAngle(angleToTarget - currentHeading);

                    axialPower = translationalPower * Math.cos(moveAngle);
                    lateralPower = translationalPower * Math.sin(moveAngle);

                    // 转向逻辑保持不变
                    double rawFarYawPower = headingError * HEADING_P_GAIN_FAR;
                    yawPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawFarYawPower));
                } else {
                    // 阶段2：近距离，仅纯旋转对准
                    axialPower = 0.0;
                    lateralPower = 0.0;
                    double rawCloseYawPower = headingError * HEADING_P_GAIN_CLOSE;
                    yawPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawCloseYawPower));
                }

                // 对防滑牵引系统施加精确的控制力度
                ATS.setTargetPower(axialPower, lateralPower, yawPower);
                break;
            case RUN_ANTILOCK_ONLY:
                break;
        }

        // 更新 ATS 系统以启用功能并处理防抱死系统相关问题。
        ATS.update();
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

    public void setTargetPos(double x, double y, double heading) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = heading;
    }

    public void setTargetPosition(double x, double y) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = odo.getHeading(); // Maintain current heading for this overload
    }

    public void setSpeedOnTargetPos(double x, double y, double yaw) {
        speedOnTargetPos[0] = x;
        speedOnTargetPos[1] = y;
        speedOnTargetPos[2] = yaw;
    }
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