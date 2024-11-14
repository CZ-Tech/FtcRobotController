package org.firstinspires.ftc.teamcode.common.drive;

import static org.firstinspires.ftc.teamcode.common.Globals.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.common.Globals.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.common.Globals.HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.Globals.P_DRIVE_GAIN;
import static org.firstinspires.ftc.teamcode.common.Globals.P_TURN_GAIN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Robot;

public enum Drivetrain {
    INSTANCE;
    private final Robot robot = Robot.INSTANCE;

    private DcMotorEx driveLeftFront = null;
    private DcMotorEx driveLeftBack = null;
    private DcMotorEx driveRightFront = null;
    private DcMotorEx driveRightBack = null;

    private double targetHeading = 0;
    private double turnSpeed = 0;
    private double headingError = 0;


    Drivetrain() {
        // lfmotor 0 \______/ 1 rfmotor
        //            |    |
        //            |    |
        // lrmotor 3 /______\ 2 rrmotor

        driveLeftFront = robot.hardwareMap.get(DcMotorEx.class, "lfmotor");  // Control Hub Motor 0
        driveRightFront = robot.hardwareMap.get(DcMotorEx.class, "rfmotor"); // Control Hub Motor 1
        driveRightBack = robot.hardwareMap.get(DcMotorEx.class, "rrmotor");  // Control Hub Motor 2
        driveLeftBack = robot.hardwareMap.get(DcMotorEx.class, "lrmotor");   // Control Hub Motor 3

        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);

        //TODO: 测试底盘跑反可能是后轮有齿轮的原因，待测试
//        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
//        driveLeftBack.setDirection(DcMotor.Direction.REVERSE);
//        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
//        driveRightBack.setDirection(DcMotor.Direction.FORWARD);

    }

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

//        powerLeftFront  *=0.5 ;
//        powerRightFront *=0.5 ;
//        powerLeftBack   *=0.5 ;
//        powerRightBack  *=0.5 ;


        // TODO: 电机测试代码，反注释下方代码进行测试。
        // 一号手柄按住share键或者back键测试
        /**
         * Xbox/PS4 Button - Motor
         *   X / ▢         - Front Left
         *   Y / Δ         - Front Right
         *   B / O         - Rear  Right
         *   A / X         - Rear  Left
         *                                    The buttons are mapped to match the wheels spatially if you
         *                                    were to rotate the gamepad 45deg°. x/square is the front left
         *                    ________        and each button corresponds to the wheel as you go clockwise
         *                   / ______ \
         *     ------------.-'   _  '-..+              Front of Bot
         *              /   _  ( Y )  _  \                  ^
         *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
         *        ___  '.      ( A )     /|       Wheel       \      Wheel
         *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
         *     |       |                 |                      \
         *      '.___.' '.               |          Rear Left    \   Rear Right
         *               '.             /             Wheel       \    Wheel
         *                \.          .'              (A/X)        \   (B/O)
         *                  \________/
         *  https://rr.brott.dev/docs/v1-0/tuning/
         */
//        if (robot.gamepad1.share && IS_DEBUG) {
//            powerLeftFront = robot.gamepad1.x ? 0.3 : 0;
//            powerRightFront = robot.gamepad1.y ? 0.3 : 0;
//            powerLeftBack = robot.gamepad1.a ? 0.3 : 0;
//            powerRightBack = robot.gamepad1.b ? 0.3 : 0;
//        }

        // Use existing function to drive both wheels.
        setDrivePower(powerLeftFront, powerRightFront, powerLeftBack, powerRightBack);
    }


    private void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        driveLeftFront.setPower(leftFrontPower);
        driveRightFront.setPower(rightFrontPower);
        driveLeftBack.setPower(leftBackPower);
        driveRightBack.setPower(rightBackPower);
    }

    /**
     * 停止电机
     *
     * @return null
     */
    private void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }


    private void setRunMode(DcMotor.RunMode mode) {
        driveLeftFront.setMode(mode);
        driveRightFront.setMode(mode);
        driveLeftBack.setMode(mode);
        driveRightBack.setMode(mode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

    private void setTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFront.setTargetPosition(driveLeftFront.getCurrentPosition() + moveCounts);
        driveRightFront.setTargetPosition(driveRightFront.getCurrentPosition() + moveCounts);
        driveLeftBack.setTargetPosition(driveLeftBack.getCurrentPosition() + moveCounts);
        driveRightBack.setTargetPosition(driveRightBack.getCurrentPosition() + moveCounts);
    }

    private void setStrafeTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        driveLeftFront.setTargetPosition(driveLeftFront.getCurrentPosition() + moveCounts);
        driveRightFront.setTargetPosition(driveRightFront.getCurrentPosition() - moveCounts);
        driveLeftBack.setTargetPosition(driveLeftBack.getCurrentPosition() - moveCounts);
        driveRightBack.setTargetPosition(driveRightBack.getCurrentPosition() + moveCounts);
    }

    private boolean isAllBusy() {
        return driveLeftFront.isBusy() && driveLeftBack.isBusy() && driveRightFront.isBusy() && driveRightBack.isBusy();
    }

    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    private double getHeading(AngleUnit unit) {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        AngularVelocity velocity = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//        telemetry.addData("xyz", "%.2f %.2f %.2f", velocity.xRotationRate, velocity.yRotationRate, velocity.zRotationRate);
//        telemetry.addData("Yaw/Pitch/Roll", orientation.toString());
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        return orientation.getYaw(unit);
    }

    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public Drivetrain resetYaw() {
        robot.imu.resetYaw();
        targetHeading = 0;
        return this;
    }


    public Drivetrain driveStraight(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            driveRobot(speed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                driveRobot(speed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public Drivetrain driveStrafe(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStrafeTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            speed = Math.abs(speed);
            driveRobot(0, speed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                driveRobot(0, speed, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public Drivetrain rush(double distance, double heading) {

        // Ensure that the OpMode is still active
        if (robot.opMode.opModeIsActive()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop

            double driveSpeed = 0;
            double tempSpeed = 0;
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            // keep looping while we are still active, and BOTH motors are running.
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -1, 1);
                if (runtime.seconds() < 1) {
                    driveSpeed = runtime.seconds();
                } else {
                    driveSpeed = 1;
                }
                tempSpeed = Math.abs(moveCounts - driveLeftFront.getCurrentPosition()) / COUNTS_PER_MOTOR_REV;
                if (tempSpeed < 1) driveSpeed = tempSpeed;
                // Apply the turning correction to the current driving speed.
                driveRobot(driveSpeed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public Drivetrain sleep(long milliseconds) {
        robot.opMode.sleep(milliseconds);
        return this;
    }

    public Drivetrain turnToHeading(double heading, double speed, double timeout, double P_DRIVE_GAIN) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (robot.opMode.opModeIsActive()
                && (Math.abs(headingError) > HEADING_THRESHOLD)
                && Math.abs(robot.gamepad1.right_stick_x) < 0.1
                && !robot.gamepad1.dpad_down
                && !robot.gamepad1.dpad_up
                && runtime.seconds() < timeout
        ) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, -turnSpeed);
//            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
//            telemetry.update();
        }

        // Stop all motion;
        stopMotor();
        return this;
    }

    public Drivetrain holdHeading(double heading, double speed, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (robot.opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, -turnSpeed);

        }

        // Stop all motion;
        stopMotor();
        return this;
    }


}
