package org.firstinspires.ftc.teamcode.common.drivetrain;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleSupplier;

/**
 * 抽象底盘 —— 封装与底盘类型无关的通用驱控逻辑。
 * <p>
 * 维护两套头向体系：
 * <ul>
 *   <li><b>原始头向</b>（raw heading）—— 来自 {@link MixedOdo} 的场地绝对角，供自动使用</li>
 *   <li><b>操作头向</b>（effective heading）—— 原始头向 + 操作手偏移，供 TeleOp 使用</li>
 * </ul>
 * 两套头向互不干扰：{@link #resetYaw()} 只修改操作头向偏移，不影响场地绝对坐标。
 * <p>
 * 子类需实现：
 * <ul>
 *   <li>{@link #driveRobot(double, double, double)} —— 底盘特有运动学</li>
 *   <li>{@link #setMode()} —— 将电机设置为驱控就绪状态</li>
 *   <li>{@link #stopMotor()} —— 停止所有电机</li>
 * </ul>
 *
 * @see MecanumChassis
 */
public abstract class OdoDrivetrain {

    private DoubleSupplier rawHeadingSource = () -> 0.0;

    /** TeleOp 头向偏移量：操作手复位"前进方向"时修改此值 */
    private double headingOffset = 0.0;

    // ==========================================
    // 朝向源与偏移管理
    // ==========================================

    /** 设置原始头向数据源（通常来自 {@link MixedOdo#getHeadingRadians()}）。 */
    public void setHeadingSource(@NonNull DoubleSupplier source) {
        this.rawHeadingSource = source;
    }

    /**
     * 复位操作手"前进"方向。
     * <p>
     * 将当前朝向设为"前进"，后续操作头向相对于此方向。
     * 不影响原始头向（自动阶段使用的场地绝对角）。
     */
    public void resetYaw() {
        headingOffset = -rawHeadingSource.getAsDouble();
    }

    /**
     * 获取操作头向 = 原始头向 + 操作手偏移（弧度）。
     * 用于 TeleOp 场向驾驶。
     */
    private double getTeleOpHeading() {
        double h = rawHeadingSource.getAsDouble() + headingOffset;
        // 归一化到 (-PI, PI]
        while (h > Math.PI) h -= 2 * Math.PI;
        while (h <= -Math.PI) h += 2 * Math.PI;
        return h;
    }

    /**
     * 执行底盘驱控。由子类实现具体的运动学算法。
     * <p>
     * 三个分量均归一化到 [-1.0, 1.0]：
     *
     * @param axial   前进/后退（正值 = 前进）
     * @param lateral 横移（正值 = 右移）
     * @param yaw     转向（正值 = 顺时针）
     */
    public abstract void driveRobot(double axial, double lateral, double yaw);

    /** 将电机设置为驱控就绪模式（如 RUN_WITHOUT_ENCODER + BRAKE）。 */
    public abstract void setMode();

    /** 设置电机运行模式（如 RUN_WITHOUT_ENCODER、RUN_USING_ENCODER 等）。 */
    public abstract void setRunMode(DcMotor.RunMode mode);

    /** 停止所有电机。 */
    public abstract void stopMotor();

    // ==========================================
    // 场向驱控（三套入口）
    // ==========================================

    /** 场向驱控 —— <b>原始头向</b>（自动阶段）。 */
    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        driveRobotFieldCentric(axial, lateral, yaw, rawHeadingSource.getAsDouble());
    }

    /** 场向驱控 —— <b>操作头向</b>（TeleOp）。 */
    public void driveRobotFieldCentricWithTeleOpHeadReset(double axial, double lateral, double yaw) {
        driveRobotFieldCentric(axial, lateral, yaw, getTeleOpHeading());
    }

    /**
     * 场向驱控 —— 显式头向。
     * @param headingRad 朝向角（弧度），逆时针为正
     */
    public void driveRobotFieldCentric(double axial, double lateral, double yaw, double headingRad) {
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);
        double rotX = lateral * cos - axial * sin;
        double rotY = lateral * sin + axial * cos;
        driveRobot(rotY, rotX, yaw);
    }


    public abstract void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);
}