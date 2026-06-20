package org.firstinspires.ftc.teamcode.solutions.chassis;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.drivetrain.OdoDrivetrain;

/**
 * 麦轮（Mecanum）底盘 —— 四轮独立驱动，全向运动。
 *
 * <p>运动学（Clamp Denominator 归一化）：
 * <pre>{@code
 * LF = ( axial + lateral + yaw) / D
 * RF = ( axial - lateral - yaw) / D
 * LB = ( axial - lateral + yaw) / D
 * RB = ( axial + lateral - yaw) / D
 * }</pre>
 */
public class MecanumChassis extends OdoDrivetrain {

    private final DcMotorEx[] motors;

    public MecanumChassis(@NonNull DcMotorEx lf, @NonNull DcMotorEx rf,
                          @NonNull DcMotorEx lb, @NonNull DcMotorEx rb) {
        this.motors = new DcMotorEx[]{lf, rf, lb, rb};
        // 初始化电机模式
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // ==========================================
    // OdoDrivetrain 抽象方法实现
    // ==========================================

    @Override
    public void driveRobot(double axial, double lateral, double yaw) {
        double d = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        motors[0].setPower((axial + lateral + yaw) / d);
        motors[1].setPower((axial - lateral - yaw) / d);
        motors[2].setPower((axial - lateral + yaw) / d);
        motors[3].setPower((axial + lateral - yaw) / d);
    }

    @Override
    public void setMode() {
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void setRunMode(DcMotor.RunMode mode) {
        for (DcMotorEx m : motors) {
            m.setMode(mode);
        }
    }

    @Override
    public void stopMotor() {
        for (DcMotorEx m : motors) m.setPower(0);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx m : motors) m.setZeroPowerBehavior(behavior);
    }
}