package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.command.Command;
import org.firstinspires.ftc.teamcode.common.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.GamepadEx;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;
import org.firstinspires.ftc.teamcode.common.vision.Vision;

public final class Robot {
    private static final Robot INSTANCE = new Robot();

    private Robot() {}

    public static Robot getInstance() {
        return INSTANCE;
    }
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Vision vision;
    public LinearOpMode opMode;
    public Drivetrain drivetrain;
    public Subsystem subsystem;
    public Command command;
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;
    public IMU imu;
    public Alliance teamColor;
    public OpModeState opModeState;

    public GoBildaPinpointDataAsync odo;

    /**
     * 初始化机器人
     *
     * @param opMode 在 opMode 里通过 robot.init(this); 进行初始化。
     */
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap; //硬件映射
        this.telemetry = opMode.telemetry; //DH上的遥测
        this.vision = Vision.INSTANCE; //视觉模块
        this.drivetrain = Drivetrain.INSTANCE; //机器人控制函数
        this.subsystem = Subsystem.INSTANCE; //上层子系统
        this.gamepad1 = new GamepadEx(opMode.gamepad1); //一个控制器
        this.gamepad2 = new GamepadEx(opMode.gamepad2); //另一个控制器
        this.command = Command.INSTANCE;

        //获取并初始化pinpoint
        this.odo = new GoBildaPinpointDataAsync(hardwareMap.get(GoBildaPinpointDriver.class, "odo"));
        odo.setOffsets(Globals.X_OFFSET, Globals.Y_OFFSET);//148.027, -68.020
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();


        //获取并初始化陀螺仪
        imu = hardwareMap.get(IMU.class, "imu"); //获取陀螺仪
        imu.initialize(new IMU.Parameters(Globals.orientationOnRobot)); //初始化陀螺仪
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
     * ps: sync不是同步的意思吗，async是异步
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
