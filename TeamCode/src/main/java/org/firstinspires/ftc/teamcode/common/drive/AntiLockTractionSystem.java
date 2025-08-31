package org.firstinspires.ftc.teamcode.common.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

/**
 * 一个用于控制电机牵引力保证加速度最大并防止打滑，同时提供带有ABS的主动刹车
 * 该类实现过程中采用0对象分配循环等方式已经进行了几乎极限的优化，修改时请不要在update()内创建对象！
 */
public class AntiLockTractionSystem {

    // 定义单位转换常量
    private static final double INCH_TO_MM = 25.4;

    public static final short MOTORS_COUNT = 4;

    private final ATSConfig config;

    public static class ATSConfig {
        public final double ATS_START_POWER;  // ATS加速起始功率，增大能提升启动速度，
        // 但可能导致初始打滑
        // ATS系统防滑灵敏度，预期轮转速和实际之差大于该值时判定为打滑，放大后电机调度会更激进，但也更容易打滑
        public final double ATS_LOCK_THRESHOLD;  // 单位RPS
        // ATS系统打滑时自动功率增减系数，正数即可，该值乘实际轮转速与理想轮转速之差即得功率增减值
        // 这个值的理想范围理论上在一定范围内随防滑灵敏度增大而减小，建议不要太大，否则...估计也出不了啥大逝()
        public final double ANTISLIP_POWER_CHANGE_FACTOR;
        // ATS系统未打滑时步进加减功率步长(0~1, 单位长度意义与功率数字相同)（预估效果同上（））
        public final double ATS_POWER_CHANGE_STEP;
        public final double LATERAL_DISTANCE;  // 左右轮距TODO(单位似乎是英寸)调试此值让机器走准
        public final double WHEELBASE;  // 前后轴距，单位英寸(虽然实际调用时使转换成毫米用的)  //TODO:数据是我胡编的，得调（）

        public final double WHEEL_DIAMETER_INCHES;   // 车轮直径（单位：英寸），用于计算车轮周长

        public final double INV_COUNTS_PER_MOTOR_REV;  // 电机旋转一周编码器增加的值的倒数

        private ATSConfig(Builder builder) {
            this.ATS_START_POWER = builder.ATS_START_POWER;
            this.ATS_LOCK_THRESHOLD = builder.ATS_LOCK_THRESHOLD;
            this.ANTISLIP_POWER_CHANGE_FACTOR = builder.ANTISLIP_POWER_CHANGE_FACTOR;
            this.ATS_POWER_CHANGE_STEP = builder.ATS_POWER_CHANGE_STEP;
            this.LATERAL_DISTANCE = builder.LATERAL_DISTANCE;
            this.WHEELBASE = builder.WHEELBASE;
            this.INV_COUNTS_PER_MOTOR_REV = 1.0 / builder.COUNTS_PER_MOTOR_REV;
            this.WHEEL_DIAMETER_INCHES = builder.WHEEL_DIAMETER_INCHES;
        }

        public static Builder getBuilder() {
            return new Builder();
        }

        public static class Builder {
            private double ATS_START_POWER = 0.8;
            private double ATS_LOCK_THRESHOLD = 0.1;
            private double ANTISLIP_POWER_CHANGE_FACTOR = 0.25;
            private double ATS_POWER_CHANGE_STEP = 0.1;
            private double LATERAL_DISTANCE = 10.63;
            private double WHEELBASE = 10.63;
            private double WHEEL_DIAMETER_INCHES = 4.00;
            private double COUNTS_PER_MOTOR_REV = 537.7;

            /**
             * 设置ATS加速起始功率。
             * 增大能提升启动速度，但可能导致初始打滑。
             * @param atsStartPower ATS加速起始功率
             * @return Builder
             */
            public Builder setAtsStartPower(double atsStartPower) {
                this.ATS_START_POWER = atsStartPower;
                return this;
            }

            /**
             * 设置ATS系统防滑灵敏度。
             * 预期轮转速和实际之差大于该值时判定为打滑，放大后电机调度会更激进，但也更容易打滑。
             * @param atsLockThreshold ATS系统防滑灵敏度 (单位RPS)
             * @return Builder
             */
            public Builder setAtsLockThreshold(double atsLockThreshold) {
                this.ATS_LOCK_THRESHOLD = atsLockThreshold;
                return this;
            }

            /**
             * 设置ATS系统打滑时自动功率增减系数。
             * 正数即可，该值乘实际轮转速与理想轮转速之差即得功率增减值。
             * 这个值的理想范围理论上在一定范围内随防滑灵敏度增大而减小，建议不要太大。
             * @param antislipPowerChangeFactor ATS系统打滑时自动功率增减系数
             * @return Builder
             */
            public Builder setAntislipPowerChangeFactor(double antislipPowerChangeFactor) {
                this.ANTISLIP_POWER_CHANGE_FACTOR = antislipPowerChangeFactor;
                return this;
            }

            /**
             * 设置ATS系统未打滑时步进加减功率步长。
             * @param atsPowerChangeStep ATS系统未打滑时步进加减功率步长 (0~1, 单位长度意义与功率数字相同)
             * @return Builder
             */
            public Builder setAtsPowerChangeStep(double atsPowerChangeStep) {
                this.ATS_POWER_CHANGE_STEP = atsPowerChangeStep;
                return this;
            }

            /**
             * 设置左右轮距。
             * @param lateralDistance 左右轮距 (单位似乎是英寸)
             * @return Builder
             */
            public Builder setLateralDistance(double lateralDistance) {
                this.LATERAL_DISTANCE = lateralDistance;
                return this;
            }

            /**
             * 设置前后轴距。
             * @param wheelbase 前后轴距 (单位英寸)
             * @return Builder
             */
            public Builder setWheelbase(double wheelbase) {
                this.WHEELBASE = wheelbase;
                return this;
            }

            /**
             * 设置车轮直径。
             * @param wheelDiameterInches 车轮直径 (单位：英寸)
             * @return Builder
             */
            public Builder setWheelDiameterInches(double wheelDiameterInches) {
                this.WHEEL_DIAMETER_INCHES = wheelDiameterInches;
                return this;
            }

            /**
             * 设置每电机转的计数。
             * @param countsPerMotorRev 每电机转的计数
             * @return Builder
             */
            public Builder setCountsPerMotorRev(double countsPerMotorRev) {
                this.COUNTS_PER_MOTOR_REV = countsPerMotorRev;
                return this;
            }

            public ATSConfig build() {
                return new ATSConfig(this);
            }
        }
    }

    // 计算底盘尺寸几何因子 (L + W)，并转换为毫米
    // 这个因子代表了旋转对车轮线速度的贡献系数
    // L = 左右轮距的一半
    // W = 前后轮距的一半
    // 注：该公式仅当每个轮子距离机器人几何中心的距离就是 (L/2 + W/2)，且轮上小轮与垂直方向夹角就是45°，此模型准确
    public final double geometryFactor_mm;
    // 车轮周长(mm)
    public final double INV_WHEEL_CIRCUMFERENCE;


    // 以下电机相关数组储存顺序均为
    // 0：LeftFront
    // 1：RightFront
    // 2：RightBack
    // 3：LeftBack
    public final DcMotorEx[] motors = new DcMotorEx[MOTORS_COUNT];
    private final double[] targetPower = new double[MOTORS_COUNT];  // 目标电机功率
    private final double[] currentPower = new double[MOTORS_COUNT];  // 暂存当前电机功率

    private final GoBildaPinpointDataAsync odo;


    // --- 用于复用内存的成员变量，保证高速循环期间0对象创建防止GC暂停影响检测频率 ---
    private final double[] wheelRPS = new double[MOTORS_COUNT];
    private final double[] groundRPS = new double[MOTORS_COUNT];

    private short isSlippingCount = 0;

    public AntiLockTractionSystem(GoBildaPinpointDataAsync odo, ATSConfig config) {
        this.odo = odo;
        this.config = config;

        motors[0] = Robot.getInstance().hardwareMap.get(DcMotorEx.class, "lfmotor");  // 电机 0 （左前）
        motors[1] = Robot.getInstance().hardwareMap.get(DcMotorEx.class, "rfmotor");  // 电机 1 （右前）
        motors[2] = Robot.getInstance().hardwareMap.get(DcMotorEx.class, "rrmotor");  // 电机 2 （右后）
        motors[3] = Robot.getInstance().hardwareMap.get(DcMotorEx.class, "lrmotor");  // 电机 3 （左后）
        // 右后电机连接在接口2上！！ ;<

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        geometryFactor_mm =
                (config.LATERAL_DISTANCE / 2.0 + config.WHEELBASE / 2.0) * INCH_TO_MM;
        INV_WHEEL_CIRCUMFERENCE = 1 / (config.WHEEL_DIAMETER_INCHES * INCH_TO_MM * Math.PI);

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * 设置电机期望达到的目标功率。
     * 程序将尽量以最短的时间达到并维持此功率，加速过程中会自动防止打滑。
     * 程序会自动调节超出范围的功率并按比例化归，保证行动方向正确。
     * @param axial 前后方向目标功率(-1~1). 正数为前进, 负数为后退.
     * @param lateral 左右方向目标功率(-1~1). 正数为右转, 负数为左转.
     * @param yaw 旋转目标功率(-1~1). 正数为顺时针旋转, 负数为逆时针旋转.
     */
    public void setTargetPower(double axial, double lateral, double yaw) {
        double invDenominator = 1 / (Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1));
        // 化除为乘提升效率
        targetPower[0] = (axial + lateral + yaw) * invDenominator;
        targetPower[1] = (axial - lateral - yaw) * invDenominator;
        targetPower[2] = (axial + lateral - yaw) * invDenominator;
        targetPower[3] = (axial - lateral + yaw) * invDenominator;
    }

    /**
     * 这个方法需要在循环中被高频周期性调用。
     * 注：该方法!不!会更新IMU数据！！
     */
    public void update() {
        updateWheelRPS();
        updateGroundRPS();
        double minReduceRatio = 1;
        double currentReduceRatio;
        isSlippingCount = 0;
        for (int i=0; i<MOTORS_COUNT; i++) {  // 分别遍历处理四个轮子
            double slipSpeeds = wheelRPS[i] - groundRPS[i];  // 实际轮速与预期轮速之差，正说明加速时打滑
                                                             // 负说明减速时打滑
            if (Math.abs(slipSpeeds) >= config.ATS_LOCK_THRESHOLD) {  // 若大于阈值则判定为打滑
                isSlippingCount++;
                /* 视情况调整功率（注：此处运行时实际可加可减，视上面计算出的差速正负而定，理论上能够处理一切打滑情况）
                 * 以下是公式说明：
                 * slipSpeeds = 实际轮速(RPS) - 期望地速(RPS)
                 *
                 * 1. 加速打滑 (Traction Control): 此时 |wheelSpeed| 一定大于 |groundSpeed|
                 *    此时，轮子转得比地面快。我们需要减小驱动力。
                 *    - 正向需减小功率数值: wheel > ground > 0  => slipSpeeds 为正。
                 *    - 反向需增大功率数值(绝对值减小): wheel < ground < 0  => slipSpeeds 为负。
                 *
                 * 2. 刹车抱死 (ABS):
                 *    此时，轮子转得比地面慢（或已被锁死）。我们需要减小刹车力。
                 *    - 正向需增大功率数值: 0 < wheel < ground  => slipSpeeds 为负。
                 *    - 反向需减小功率数值(绝对值增大): 0 > wheel > ground  => slipSpeeds 为正。
                 *
                 * `currentPower - slipSpeeds * factor` 这公式能同时正确处理以上四种情况：
                 * - 当 slipSpeeds > 0 (正向加速打滑 或 反向刹车抱死), 减小功率值。
                 *   即减小正向驱动力 或 减小反向刹车力。
                 * - 当 slipSpeeds < 0 (反向加速打滑 或 正向刹车抱死), 增加功率值。
                 *   即减小反向驱动力 或 减小正向刹车力。
                 *
                 * 最终效果：功率调整总是朝着减小 |slipSpeeds| 的方向进行，使轮速趋近地速。
                 */
                currentReduceRatio =
                        clampPower(currentPower[i] - slipSpeeds * config.ANTISLIP_POWER_CHANGE_FACTOR)
                                / targetPower[i];  // 是的你没猜错，这里可能会产生±Infinity
                if (currentReduceRatio < minReduceRatio) {
                    minReduceRatio = currentReduceRatio;
                }
            } else {
                double powerGap = targetPower[i] - currentPower[i];
                if (Math.abs(powerGap) > 1e-9) { // 用小阈值判断浮点数是否接近零
                    double powerStep = Math.min(config.ATS_POWER_CHANGE_STEP, Math.abs(powerGap));
                    // 下面if中的clampPower理论上不是必须的，但出于浮点数精度和防御性编程考虑还是加上为妙
                    if (powerGap > 0) {
                        currentReduceRatio = clampPower(currentPower[i] + powerStep) / targetPower[i];  // 是的你没猜错，这里可能会产生±Infinity
                    } else {  // powerGap < 0
                        currentReduceRatio = clampPower(currentPower[i] - powerStep) / targetPower[i];  // 是的你没猜错，这里可能会产生±Infinity
                    }
                    if (currentReduceRatio < minReduceRatio) {
                        minReduceRatio = currentReduceRatio;
                    }
                }
            }
        }
        for (int i=0; i<MOTORS_COUNT; i++) {
            if (Math.abs(targetPower[i]) <= 1e-9) {
                currentPower[i] = 0;  // 防止NaN
            } else {
                currentPower[i] = clampPower(targetPower[i] * minReduceRatio);  // clampPower以消除±Infinity
            }
            motors[i].setPower(currentPower[i]);
        }
    }


    public void updateWheelRPS() {
        wheelRPS[0] = motors[0].getVelocity() * config.INV_COUNTS_PER_MOTOR_REV;
        wheelRPS[1] = motors[1].getVelocity() * config.INV_COUNTS_PER_MOTOR_REV;
        wheelRPS[2] = motors[2].getVelocity() * config.INV_COUNTS_PER_MOTOR_REV;
        wheelRPS[3] = motors[3].getVelocity() * config.INV_COUNTS_PER_MOTOR_REV;
    }

    /**
     * 获取根据地速计算出不打滑情况下的四轮转速（单位RPS，即每秒旋转圈数）
     * 结果直接写回成员变量groundRPS避免对象创建
     */
    public void updateGroundRPS() {
        calculateWheelSpeeds(
                odo.getVelX(),
                odo.getVelY(),
                odo.getHeadingVelocity(),
                groundRPS);
        for (int i = 0; i < MOTORS_COUNT; i++) {
            groundRPS[i] *= INV_WHEEL_CIRCUMFERENCE;
        }
    }

    /**
     * 根据机器人的期望速度和角速度，计算四个麦克纳姆轮各自需要的切向速度。
     * 这个方法使用了您在 Globals 类中定义的机器人几何尺寸。
     *
     * @param axial_mmps   机器人的轴向（前进/后退）期望速度，单位：mm/s。
     * @param lateral_mmps 机器人的横向（左/右平移）期望速度，单位：mm/s。
     * @param yaw_radps    机器人的偏航（旋转）期望角速度，单位：rad/s。
     * @param resultMilliPs 将结果储存在这个包含四个double值的数组，
     *         依次为：[左前轮速度, 右前轮速度, 右后轮速度, 左后轮速度]，单位均为 mm/s。
     */
    public void calculateWheelSpeeds(
            double axial_mmps,
            double lateral_mmps,
            double yaw_radps,
            double[] resultMilliPs
    ) {

        // 将输入的角速度(rad/s)转换为由旋转贡献的线速度(mm/s)
        // 线速度 = 角速度 * 半径（这里的半径是 geometryFactor_mm）
        double yaw_velocity_mmps = yaw_radps * geometryFactor_mm;

        // 应用运动学逆解公式，计算每个轮子的最终线速度，并写回传入数组避免创建新对象
        resultMilliPs[0] = axial_mmps + lateral_mmps + yaw_velocity_mmps;  // 左前轮速度
        resultMilliPs[1] = axial_mmps - lateral_mmps - yaw_velocity_mmps;  // 右前轮速度
        resultMilliPs[2]  = axial_mmps + lateral_mmps - yaw_velocity_mmps; // 右后轮速度
        resultMilliPs[3] = axial_mmps - lateral_mmps + yaw_velocity_mmps;  // 左后轮速度
    }

    /**
     * 将一个值限制在 [-1.0, 1.0] 的范围内
     * 若超出范围则取与原值临近的-1或1
     */
    public static double clampPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * 该方法用于设置四个电机的运行模式.
     * @param mode 电机运行模式.
     */
    public void setRunMode(DcMotor.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public boolean isUsingLimitingFriction(int minTrueMotorCount) {
        short trueCount = 0;
        for (short i = 0; i < MOTORS_COUNT; i++) {
            if (targetPower[i] - currentPower[i] > 1e-9) trueCount++;
        }
        return trueCount >= minTrueMotorCount;
    }

    public boolean isSlipping() {
        return isSlippingCount > 0;
    }

    public void autoBreak() {
        setTargetPower(-1, -1, 0);
    }
}
