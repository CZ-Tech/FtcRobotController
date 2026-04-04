package org.firstinspires.ftc.teamcode.common.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;


public class OdoDrivetrain {
    private final Robot robot;

    public DcMotorEx driveLeftFront = null;
    public DcMotorEx driveLeftBack = null;
    public DcMotorEx driveRightFront = null;
    public DcMotorEx driveRightBack = null;
    public static double angleOffset = 0;
    public double tempAngle = 0;
    public double targetAngle = 0;
    public static double TURN_RANGE = 5;
    public static double TURN_SPEED = 0.4;

    private double HEADING_ERROR = 0;

    public OdoDrivetrain(Robot robot) {
        this.robot = robot;


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

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    private double reCulcGamepad(double v) {
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.09;//0.09是23-24赛季底盘启动需要的功率
        } else if (v < 0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.87 * v * v * v - 0.09; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }

    public void setShootAngle(double angle){
        targetAngle = angle;
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
//        robot.odo.update();
//        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
//        AngularVelocity velocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//        return orientation.getYaw(angleUnit);
        return angleUnit.fromDegrees(robot.odo.getHeading(AngleUnit.DEGREES) + angleOffset);
    }

//    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
////        robot.odo.update();
//        return unnormalizedAngleUnit.fromDegrees(robot.odo.getHeading(unnormalizedAngleUnit) + angleOffset);
//    }

    public void resetYaw() {
        tempAngle = getHeading(AngleUnit.DEGREES);
        angleOffset = AngleUnit.DEGREES.normalize(angleOffset - tempAngle);

//        robot.gyroTracker.handleReset(UnnormalizedAngleUnit.DEGREES);  // FiXME 理论上这玩意没用了
        while(robot.opMode.gamepad1.share || robot.opMode.gamepad2.options);
    }


    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);
        robot.telemetry.addData("heading", botHeading);
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

    public void turnTo(double heading, double turn_range, double speed){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        robot.odo.update();
        HEADING_ERROR = getHeading(AngleUnit.DEGREES) - heading;
        while(robot.opMode.opModeIsActive() && runtime.seconds() <= 4 && Math.abs(HEADING_ERROR) > turn_range){
            robot.odo.update();

//            if (HEADING_ERROR <= 150) HEADING_ERROR = getHeading(AngleUnit.DEGREES) - heading;
            HEADING_ERROR = getHeading(AngleUnit.DEGREES) - heading;

//            robot.telemetry.addData("Unnormalized Heading", getHeading(UnnormalizedAngleUnit.DEGREES));
            robot.telemetry.addData("Normalized Heading", getHeading(AngleUnit.DEGREES));
            robot.telemetry.addData("Error", HEADING_ERROR);
            robot.telemetry.addData("Time", runtime.seconds());
            robot.telemetry.update();

            driveRobotFieldCentric(
                    reCulcGamepad(-robot.opMode.gamepad1.left_stick_y),
                    reCulcGamepad(robot.opMode.gamepad1.left_stick_x),
                    Range.clip(HEADING_ERROR * Globals.TURN_GAIN, -speed, speed));
        }
        stopMotor();
    }

    public void turnTo(double heading){
        turnTo(heading, TURN_RANGE, TURN_SPEED);
    }

    public void turnTo(double heading, double turn_range){
        turnTo(heading, turn_range, TURN_SPEED);
    }
}
