package org.firstinspires.ftc.teamcode.common.drive;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class OdoDrivetrain {
    private final Robot robot;

    public DcMotorEx driveLeftFront = null;
    public DcMotorEx driveLeftBack = null;
    public DcMotorEx driveRightFront = null;
    public DcMotorEx driveRightBack = null;
    public double angleOffset = 0;

    public OdoDrivetrain(Robot robot) {
        this.robot = robot;
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 确保下方与configure里设置的一致
        driveLeftFront = robot.hardwareMap.get(DcMotorEx.class, Globals.LeftFrontMotor);
        driveRightFront = robot.hardwareMap.get(DcMotorEx.class, Globals.RightFrontMotor);
        driveRightBack = robot.hardwareMap.get(DcMotorEx.class, Globals.RightBackMotor);
        driveLeftBack = robot.hardwareMap.get(DcMotorEx.class, Globals.LeftBackMotor);

        // 确保控制4个电机正转向前。
        driveLeftFront.setDirection(Globals.LeftFrontMotorDirection);
        driveLeftBack.setDirection(Globals.LeftBackMotorDirection);
        driveRightFront.setDirection(Globals.RightFrontMotoDirection);
        driveRightBack.setDirection(Globals.RightBackMotorDirection);

    }

    public static double VOLTAGE = 12;

    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double powerLeftFront = (axial + lateral + yaw) / denominator;
        double powerRightFront = (axial - lateral - yaw) / denominator;
        double powerLeftBack = (axial - lateral + yaw) / denominator;
        double powerRightBack = (axial + lateral - yaw) / denominator;


        // TODO: 电机测试代码。测试完可以注释掉。
        if (Globals.DEBUG && robot.opMode.gamepad1.share) {
            powerLeftFront = robot.opMode.gamepad1.x ? 0.3 : 0;
            powerRightFront = robot.opMode.gamepad1.y ? 0.3 : 0;
            powerLeftBack = robot.opMode.gamepad1.a ? 0.3 : 0;
            powerRightBack = robot.opMode.gamepad1.b ? 0.3 : 0;
        }

        // Use existing function to drive both wheels.
        robot.telemetry.addData("LF", powerLeftFront);
        robot.telemetry.addData("RF", powerRightFront);
        robot.telemetry.addData("LB", powerLeftBack);
        robot.telemetry.addData("LR", powerRightBack);

        double MOTOR_GAIN = Math.abs(VOLTAGE / robot.getVoltage());
        setDrivePower(powerLeftFront, powerRightFront, powerLeftBack, powerRightBack);
    }

    public double getHeading(AngleUnit angleUnit) {
        return robot.odo.getHeading(angleUnit) + angleOffset;
    }

    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return robot.odo.getHeading(unnormalizedAngleUnit) + angleOffset;
    }

    public void resetYaw() {
        robot.odo.recalibrateIMU();
    }

    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    public void setDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        driveLeftFront.setPower(leftFrontPower);
        driveRightFront.setPower(rightFrontPower);
        driveLeftBack.setPower(leftBackPower);
        driveRightBack.setPower(rightBackPower);
    }

    public void stopMotor() {
        setDrivePower(0);
    }

    public void setRunMode(DcMotor.RunMode mode) {
        driveLeftFront.setMode(mode);
        driveRightFront.setMode(mode);
        driveLeftBack.setMode(mode);
        driveRightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }
}
