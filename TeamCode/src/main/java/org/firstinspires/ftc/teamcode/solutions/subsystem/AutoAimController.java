package org.firstinspires.ftc.teamcode.solutions.subsystem;

import static org.firstinspires.ftc.teamcode.common.Globals.HEAD_SHOOT_ANGLE_DEG;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.drivetrain.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drivetrain.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.util.MixedTelemetry;

@Config
public class AutoAimController {

    public static double TURN_KP = 0.015;
    public static double TURN_KI = 0.01;
    public static double TURN_KD = 0.0015;
    public static double MAX_YAW_POWER = 1.0;

    // EMA 低通滤波系数 (范围 0 到 1)
    // 值越小，滤波越强，对震动的抵抗力越好，但 D 的反应越慢
    // 值越大，滤波越弱，D 的反应越快
    public static double D_FILTER_ALPHA = 0.6;

    private final MixedTelemetry telemetry;
    private final MixedOdo odo;
    private final OdoDrivetrain drivetrain;

    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0.0;
    private double lastRawError = 0.0;
    private double lastFilteredDerivative = 0.0;
    private boolean isFirstRun = true;
    private final double MAX_INTEGRAL_SUM = 20.0;

    public AutoAimController(MixedTelemetry telemetry, MixedOdo odo, OdoDrivetrain drivetrain) {
        this.telemetry = telemetry;
        this.odo = odo;
        this.drivetrain = drivetrain;
    }

    /**
     * 纯 PID 航向控制——只计算输出功率，不操作底盘。
     * 供 PinpointTrajectory 等需要自行处理驱动逻辑的调用方使用。
     */
    public double getPowerToAngle(double currDeg, double targetDeg) {

        // 1. 获取原始误差
        double rawHeadingError = targetDeg - currDeg;

        // 2. 角度标准化 (最短路径)
        while (rawHeadingError > 180.0) { rawHeadingError -= 360.0; }
        while (rawHeadingError <= -180.0) { rawHeadingError += 360.0; }

        // 3. 时间计算
        double dt = timer.seconds();
        timer.reset();
        if (dt < 0.005) dt = 0.005; // 防除零爆炸

        if (isFirstRun || dt >= 0.5) {
            resetController();
            lastRawError = rawHeadingError;
            dt = 0.01;
            isFirstRun = false;
            lastFilteredDerivative = 0.0;
        }

        // ==========================================
        // 4. 计算 P, I, D
        // ==========================================

        // --- 比例项 P (使用无延迟的原始误差) ---
        double pTerm = TURN_KP * rawHeadingError;

        // --- 积分项 I ---
        if (Math.abs(rawHeadingError) < 20.0) {
            integralSum += rawHeadingError * dt;
        } else {
            integralSum = 0;
        }
        integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
        double iTerm = TURN_KI * integralSum;

        // --- 微分项 D (对变化率进行 EMA 低通滤波) ---
        double rawDerivative = (rawHeadingError - lastRawError) / dt;
        double filteredDerivative = (D_FILTER_ALPHA * rawDerivative) + ((1.0 - D_FILTER_ALPHA) * lastFilteredDerivative);
        double dTerm = TURN_KD * filteredDerivative;

        // ==========================================
        // 5. 组合输出
        // ==========================================
        double yawPower;

        if (Math.abs(rawHeadingError) < 1.0) {
            yawPower = 0.0;
            integralSum = 0.0;
        } else {
            yawPower = pTerm + iTerm + dTerm;
        }

        // 更新历史状态
        lastRawError = rawHeadingError;
        lastFilteredDerivative = filteredDerivative;

        // 限制最大功率
        if (yawPower > MAX_YAW_POWER) yawPower = MAX_YAW_POWER;
        else if (yawPower < -MAX_YAW_POWER) yawPower = -MAX_YAW_POWER;

        return yawPower;
    }

    /**
     * 瞄准目标位置：自动计算目标航向 + PID 控制 + 驱动底盘。
     */
    public void aimAtTarget(Pose2D targetPos) {
        aimAtTarget(targetPos, 0.0, 0.0);
    }

    public void aimAtTarget(Pose2D targetPos, double currAxial, double currLateral) {

        double currentX = odo.getPosition().getX(DistanceUnit.METER);
        double currentY = odo.getPosition().getY(DistanceUnit.METER);
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);

        double dx = targetPos.getX(DistanceUnit.METER) - currentX;
        double dy = targetPos.getY(DistanceUnit.METER) - currentY;

        double targetHeading = Math.toDegrees(Math.atan2(dy, dx)) + HEAD_SHOOT_ANGLE_DEG;

        double yawPower = getPowerToAngle(currentHeading, targetHeading);

        drivetrain.driveRobotFieldCentric(currAxial, currLateral, -yawPower);

        telemetry.addData("Aim RawError", targetHeading - currentHeading);
        telemetry.addData("Aim YawPower", yawPower);
    }

    public void resetController() {
        isFirstRun = true;
        integralSum = 0;
        lastFilteredDerivative = 0;
        timer.reset();
    }
}