package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drivetrain.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drivetrain.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.GamepadBind;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.GamepadReflectBinder;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.ListEventGamepad;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers.Buttons;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.MixedTelemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Duo;

public class Robot {
    public HardwareMap hardwareMap;
    public MixedTelemetry telemetry;
    public LinearOpMode opMode;
    public OdoDrivetrain odoDrivetrain;
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;
    public Alliance teamColor;
    public MixedOdo odo;
//    public PinpointTrajectory pinpointTrajectory;


    /**
     * 初始化机器人
     *
     * @param opMode 在 opMode 里通过 robot.init(this); 进行初始化。
     */
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap; //硬件映射
        this.telemetry = new MixedTelemetry(opMode.telemetry);//DH上的遥测
        this.odo = new MixedOdo(telemetry,
                hardwareMap.get(GoBildaPinpointDriver.class, Globals.odoName),
                hardwareMap.get(Limelight3A.class, "limelight"));

        this.gamepad1 = new GamepadEx(opMode.gamepad1, opMode::opModeIsActive); //一个控制器
        this.gamepad2 = new GamepadEx(opMode.gamepad2, opMode::opModeIsActive); //另一个控制器

        this.odoDrivetrain = new OdoDrivetrain(this);

//        this.pinpointTrajectory = new PinpointTrajectory(this);

        GamepadReflectBinder.bind(this);
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
     * 停止机器人的动作，时间以毫秒为单位
     *
     * @param milliseconds 需要暂停的毫秒数量
     * @return Robot
     */
    public Robot sleep(long milliseconds) {
        opMode.sleep(milliseconds);
        return this;
    }

    /**
     * 等待一段时间，会阻塞当前进程（何意味？
     * @param millisecond
     * @return 甚至还能链式调用？？？
     */
    public Robot waitFor(double millisecond){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= millisecond && opMode.opModeIsActive());
        return this;
    }

    // 专属行为：只有在 MainTeleOp（正赛）里，B 键才是反转吐出
    @GamepadBind(gamepad = 1, button = Buttons.B, activeIn = {Duo.class})
    private void reverseIntake() {
        // ...
    }
}
