package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.command.Command;
import org.firstinspires.ftc.teamcode.common.drive.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.vision.Vision;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Vision vision;
    public LinearOpMode opMode;
    public OdoDrivetrain odoDrivetrain;
    public Subsystem subsystem;
    public Command command;
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;
    public IMU imu;
    public Alliance teamColor;
    public OpModeState opModeState;
    public GoBildaPinpointDriver odo;
    public PinpointTrajectory pinpointTrajectory;

    /**
     * 初始化机器人
     *
     * @param opMode 在 opMode 里通过 robot.init(this); 进行初始化。
     */
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap; //硬件映射
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());//DH上的遥测，使用了FTCDashboard
        this.vision = new Vision(this); //视觉模块
        this.subsystem = new Subsystem(this); //上层子系统
        this.gamepad1 = new GamepadEx(opMode.gamepad1); //一个控制器
        this.gamepad2 = new GamepadEx(opMode.gamepad2); //另一个控制器
        this.command = new Command(this); //命令系统
        this.pinpointTrajectory = new PinpointTrajectory(this);

        // 定义电机
        this.odoDrivetrain = new OdoDrivetrain(this);


        // 反注释下方代码进入调试模式，用于测试每个电机方向。
        // Globals.DEBUG = true;
        // 一号手柄按住share键或者back键测试
        /**
         * Xbox/PS4 Button - Motor
         *   X / ▢         - Left  Front
         *   Y / Δ         - Right Front
         *   B / O         - Right Back
         *   A / X         - Left  Back
         *                                    The buttons are mapped to match the wheels spatially if you
         *                                    were to rotate the gamepad 45deg°. x/square is the front left
         *                    ________        and each button corresponds to the wheel as you go clockwise
         *                   / ______ \
         *     ------------.-'   _  '-..+              Front of Bot
         *              /   _  ( Y )  _  \                  ^
         *             |  ( X )  _  ( B ) |      Left Front  \    Right Front
         *        ___  '.      ( A )     /|       Wheel       \      Wheel
         *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
         *     |       |                 |                      \
         *      '.___.' '.               |          Left Back    \   Right Back
         *               '.             /             Wheel       \    Wheel
         *                \.          .'              (A/X)        \   (B/O)
         *                  \________/
         *  https://rr.brott.dev/docs/v1-0/tuning/
         */

        // 确保下方与configure里设置的一致
        odoDrivetrain.driveLeftFront = hardwareMap.get(DcMotorEx.class, "lfm");
        odoDrivetrain.driveRightFront = hardwareMap.get(DcMotorEx.class, "rfm");
        odoDrivetrain.driveRightBack = hardwareMap.get(DcMotorEx.class, "rbm");
        odoDrivetrain.driveLeftBack = hardwareMap.get(DcMotorEx.class, "lbm");

        // 确保控制4个电机正转向前。
        odoDrivetrain.driveLeftFront.setDirection(DcMotor.Direction.REVERSE); //forward
        odoDrivetrain.driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        odoDrivetrain.driveRightFront.setDirection(DcMotor.Direction.REVERSE); //reverse
        odoDrivetrain.driveRightBack.setDirection(DcMotor.Direction.REVERSE);


        //获取并初始化pinpoint
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //  设置里程计吊舱相对于跟踪点的位置偏移
        //  @param xOffset X吊舱偏移量（毫米），中心左侧为正，右侧为负
        //  @param yOffset Y吊舱偏移量（毫米），中心前方为正，后方为负
        //  +++++++++++++++++++++++++++++++
        //  +                        |||  +
        //  +               xOffset  |||  +
        //  +             <--------->     +
        //  +             ● Center        +        FIXME:Center指的是机器的旋转中心
        //  +             |               +
        //  +             | yOffset       +
        //  +             V               +
        //  +           =====             +
        //  +           =====             +
        //  +++++++++++++++++++++++++++++++
        odo.setOffsets(0, -95.99872249, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // TODO:确认自己使用的里程计类型
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();


        // TODO:获取并初始化control hub 陀螺仪。注意修改朝向。也可以使用pinpoint的陀螺仪。
        imu = hardwareMap.get(IMU.class, "imu"); //获取陀螺仪
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ))); //初始化陀螺仪
    }

    /**
     * 获取当前电压
     *
     * @return voltage
     */
    public double getVoltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    /**
     * 以多线程运行函数
     *
     * @param commands 需要同时运行的命令（函数或者方法）
     * @return Robot
     */
    public Robot syncRun(Runnable... commands) {
        for (Runnable command : commands) {
            new Thread(command).start();
        }
        return this;
    }

    /**
     * 停止机器人的动作，时间以毫秒为单位
     *
     * @param milliseconds 需要暂停的毫秒数量
     * @return Robot
     */
    public Robot sleep(long milliseconds) {
        opMode.sleep(milliseconds);
        return this;
    }
}
