package org.firstinspires.ftc.teamcode.common.drive;

import static org.firstinspires.ftc.teamcode.common.Globals.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.common.Globals.COUNTS_PER_MOTOR_REV;
import static org.firstinspires.ftc.teamcode.common.Globals.HEADING_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.Globals.P_DRIVE_GAIN;
import static org.firstinspires.ftc.teamcode.common.Globals.P_TURN_GAIN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Robot;

public enum Drivetrain {
    INSTANCE;
    private final Robot robot = Robot.INSTANCE;

    public DcMotorEx driveLeftFront = null;
    public DcMotorEx driveLeftBack = null;
    public DcMotorEx driveRightFront = null;
    public DcMotorEx driveRightBack = null;

    private double targetHeading = 0;
    private double turnSpeed = 0;
    private double headingError = 0;


    Drivetrain() {
        // lfmotor 0 \______/ 1 rfmotor
        //            |    |
        //            |    |
        // lrmotor 3 /______\ 2 rrmotor
        // 左前电机 0 \______/ 1 右前电机
        //            |    |
        //            |    |
        // 左后电机 3 /______\ 2 右后电机

        driveLeftFront = robot.hardwareMap.get(DcMotorEx.class, "lfmotor");  // Control Hub Motor 0 // 电机 0
        driveRightFront = robot.hardwareMap.get(DcMotorEx.class, "rfmotor"); // Control Hub Motor 1 // 电机 1
        driveRightBack = robot.hardwareMap.get(DcMotorEx.class, "rrmotor");  // Control Hub Motor 2 // 电机 2
        driveLeftBack = robot.hardwareMap.get(DcMotorEx.class, "lrmotor");   // Control Hub Motor 3 // 电机 3

        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);


    }

    /**
     * 该方法用于驱动机器人底盘.
     * @param axial   控制机器人前进后退.
     * @param lateral 控制机器人向左或向右平移.
     * @param yaw     控制机器人原地转向.
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        // 通过两根摇杆的位置计算每个车轮的功率
        // 为每个驱动轮设置一个变量，方便将这些值用于输出遥测数据
        // 在计算出某个电机的目标功率超过1或-1时会把所有电机功率等比例缩小
        // 从而保证机器人始终保持正确的运动方向
        // denominator是电机功率将被缩小的倍率(分母), 它会被设为大于1的电机目标功率值或1
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
        // 使用现有函数驱动所有车轮。
        setDrivePower(powerLeftFront, powerRightFront, powerLeftBack, powerRightBack);
    }


    /**
     * 该方法用于设置四个电机的功率.
     * @param leftFrontPower  左前轮的功率.
     * @param rightFrontPower 右前轮的功率.
     * @param leftBackPower   左后轮的功率.
     * @param rightBackPower  右后轮的功率.
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Output the values to the motor drives.
        // 将值输出到电机驱动器。
        driveLeftFront.setPower(leftFrontPower);
        driveRightFront.setPower(rightFrontPower);
        driveLeftBack.setPower(leftBackPower);
        driveRightBack.setPower(rightBackPower);
    }

    /**
     * 停止电机
     */
    private void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }


    /**
     * 该方法用于设置四个电机的运行模式.
     * @param mode 电机运行模式.
     */
    public void setRunMode(DcMotor.RunMode mode) {
        driveLeftFront.setMode(mode);
        driveRightFront.setMode(mode);
        driveLeftBack.setMode(mode);
        driveRightBack.setMode(mode);
    }

    /**
     * 该方法用于设置四个电机的零功率行为.
     * @param behavior 电机的零功率行为.
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * 该方法用于设定四个电机向前或向后移动的目标位置.
     * 设定目标后应启用 RUN_TO_POSITION
     * @param moveCounts 电机要移动的编码器计数.
     */
    public void setTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        // 首先设置目标，然后打开 RUN_TO_POSITION
        driveLeftFront.setTargetPosition(driveLeftFront.getCurrentPosition() + moveCounts);
        driveRightFront.setTargetPosition(driveRightFront.getCurrentPosition() + moveCounts);
        driveLeftBack.setTargetPosition(driveLeftBack.getCurrentPosition() + moveCounts);
        driveRightBack.setTargetPosition(driveRightBack.getCurrentPosition() + moveCounts);
    }

    /**
     * 该方法用于设置四个电机向左或向右平移的目标位置.
     * @param moveCounts 电机要移动的编码器计数.
     */
    private void setStrafeTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        // 首先设置目标，然后打开 RUN_TO_POSITION
        driveLeftFront.setTargetPosition(driveLeftFront.getCurrentPosition() + moveCounts);
        driveRightFront.setTargetPosition(driveRightFront.getCurrentPosition() - moveCounts);
        driveLeftBack.setTargetPosition(driveLeftBack.getCurrentPosition() - moveCounts);
        driveRightBack.setTargetPosition(driveRightBack.getCurrentPosition() + moveCounts);
    }

    /**
     * 该方法用于检查所有四个电机是否都在忙.
     * @return 如果所有四个电机都处于忙碌状态，则返回 true，否则返回 false.
     */
    private boolean isAllBusy() {
        return driveLeftFront.isBusy() && driveLeftBack.isBusy() && driveRightFront.isBusy() && driveRightBack.isBusy();
    }

    /**
     * 该方法用于以场地为中心的模式驱动机器人.
     * @param axial   控制机器人前进后退.
     * @param lateral 控制机器人向左或向右平移.
     * @param yaw     控制机器人原地转向.
     */
    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        // 将运动方向逆着机器人旋转方向进行旋转
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    /**
     * 该方法用于获取机器人的当前航向.
     * @param unit 航向的角度单位.
     * @return 机器人的当前航向.
     */
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

    /**
     * 该方法用于获取机器人的当前航向，单位为度.
     * @return 机器人的当前航向.
     */
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }


    /**
     * 该方法用于计算保持航向所需的转向修正.
     * @param desiredHeading   期望的航向.
     * @param proportionalGain 比例增益.
     * @return 转向修正值.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry // 保存遥测数据

        // Determine the heading current error
        // 确定航向当前偏差
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        // 将航向偏差归一化到[-180°, 180°]区间
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        // 将偏差乘以增益以确定所需的转向校正/ 将结果限制在 +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * 该方法用于重置机器人的偏航角.
     * @return Drivetrain 实例.
     */
    public Drivetrain resetYaw() {
        robot.imu.resetYaw();
        targetHeading = 0;
        return this;
    }


    /**
     * 该方法用于使机器人直线行驶指定的距离.
     * @param distance 要行驶的距离.
     * @param heading  要保持的航向.
     * @param speed    行驶速度.
     * @return Drivetrain 实例.
     */
    public Drivetrain driveStraight(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        // 确保 OpMode 仍处于活动状态
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // 确定新的目标位置，并传递给电机控制器
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            // 设置所需的行驶速度（对于 RUN_TO_POSITION 必须为正）
            // 开始直线行驶，然后进入控制循环
            speed = Math.abs(speed);
            driveRobot(speed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            // 在我们仍处于活动状态且两个电机都在运行时继续循环。
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                // 确定保持航向所需的转向
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                // 如果倒车，电机校正也需要反向
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                // 将转向校正应用于当前行驶速度。
                driveRobot(speed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            // 停止所有运动并关闭 RUN_TO_POSITION（退出位置控制模式）
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    /**
     * 该方法用于使机器人平移指定的距离.
     * @param distance 要平移的距离.
     * @param heading  要保持的航向.
     * @param speed    平移速度.
     * @return Drivetrain 实例.
     */
    public Drivetrain driveStrafe(double distance, double heading, double speed) {

        // Ensure that the OpMode is still active
        // 确保 OpMode 仍处于活动状态
        if (robot.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // 确定新的目标位置，并传递给电机控制器
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStrafeTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            // 设置所需的行驶速度（对于 RUN_TO_POSITION 必须为正）
            // 开始直线行驶，然后进入控制循环
            speed = Math.abs(speed);
            driveRobot(0, speed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            // 在我们仍处于活动状态且两个电机都在运行时继续循环。
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                // 确定保持航向所需的转向
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                // 如果倒车，电机校正也需要反向
                if (distance < 0) turnSpeed *= -1.0;
                turnSpeed = Range.clip(turnSpeed, -speed, speed);
                // Apply the turning correction to the current driving speed.
                // 将转向校正应用于当前行驶速度。
                driveRobot(0, speed, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            // 停止所有运动并关闭 RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    /**
     * 该方法用于使机器人以S型曲线加速和减速的方式冲向目标.
     * @param distance 要行驶的距离.
     * @param heading  要保持的航向.
     * @return Drivetrain 实例.
     */
    public Drivetrain rush(double distance, double heading) {

        // Ensure that the OpMode is still active
        // 确保 OpMode 仍处于活动状态
        if (robot.opMode.opModeIsActive()) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller
            // 确定新的目标位置，并传递给电机控制器
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            // 设置所需的行驶速度（对于 RUN_TO_POSITION 必须为正）
            // 开始直线行驶，然后进入控制循环

            double driveSpeed = 0;
            double tempSpeed = 0;
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            // keep looping while we are still active, and BOTH motors are running.
            // 在我们仍处于活动状态且两个电机都在运行时继续循环。
            while (robot.opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                // 确定保持航向所需的转向
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                // 如果倒车，电机校正也需要反向
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
                // 将转向校正应用于当前行驶速度。
                driveRobot(driveSpeed, 0, -turnSpeed);
//                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            // 停止所有运动并关闭 RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    /**
     * 该方法用于使程序暂停指定的毫秒数.
     * @param milliseconds 要暂停的毫秒数.
     * @return Drivetrain 实例.
     */
    public Drivetrain sleep(long milliseconds) {
        robot.opMode.sleep(milliseconds);
        return this;
    }

    /**
     * 该方法用于使机器人转向指定的航向.
     * @param heading      目标航向.
     * @param speed        转向速度.
     * @param timeout      超时时间.
     * @param P_DRIVE_GAIN 比例增益.
     * @return Drivetrain 实例.
     */
    public Drivetrain turnToHeading(double heading, double speed, double timeout, double P_DRIVE_GAIN) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        // 运行一次 getSteeringCorrection() 以预先计算当前偏差
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        // 在我们仍处于活动状态且未到达航向时继续循环。
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
            // 确定保持航向所需的转向
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            // 将速度限制在允许的最大值。
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            // 通过应用转向校正在原地旋转
            driveRobot(0, 0, -turnSpeed);
//            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
//            telemetry.update();
        }

        // Stop all motion;
        // 停止所有运动；
        stopMotor();
        return this;
    }

    /**
     * 该方法用于使机器人在指定的时间内保持指定的航向.
     * @param heading  要保持的航向.
     * @param speed    转向速度.
     * @param holdTime 保持时间.
     * @return Drivetrain 实例.
     */
    public Drivetrain holdHeading(double heading, double speed, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        // 在我们还有剩余时间时继续循环。
        while (robot.opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            // 确定保持航向所需的转向
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            // 将速度限制在允许的最大值。
            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            // Pivot in place by applying the turning correction
            // 通过应用转向校正在原地旋转
            driveRobot(0, 0, -turnSpeed);

        }

        // Stop all motion;
        // 停止所有运动；
        stopMotor();
        return this;
    }

    /**
     * 该方法用于使机器人转向指定的航向.
     * @param heading 目标航向.
     * @param speed   转向速度.
     * @param timeout 超时时间.
     * @return Drivetrain 实例.
     */
    public Drivetrain turnToHeading(double heading, double speed, double timeout) {
        return turnToHeading(heading, speed, timeout, P_DRIVE_GAIN);
    }

    /**
     * 该方法用于使机器人转向指定的航向.
     * @param heading 目标航向.
     * @param speed   转向速度.
     * @return Drivetrain 实例.
     */
    public Drivetrain turnToHeading(double heading, double speed) {
        return turnToHeading(heading, speed, 5);
    }

}
