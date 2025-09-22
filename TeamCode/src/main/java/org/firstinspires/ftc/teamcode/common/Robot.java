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
        this.odoDrivetrain = new OdoDrivetrain(this);

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, Globals.odoName);
        odo.setOffsets(Globals.odoXOffset, Globals.odoYOffset, DistanceUnit.MM);
        odo.setEncoderResolution(Globals.odoType);
        odo.setEncoderDirections(Globals.odoXDirection,Globals.odoYDirection);
        odo.resetPosAndIMU();

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
