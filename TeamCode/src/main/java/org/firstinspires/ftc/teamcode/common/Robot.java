package org.firstinspires.ftc.teamcode.common;


import android.util.Log;

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
//        this.odo = new MixedOdo(telemetry,
//                hardwareMap.get(GoBildaPinpointDriver.class, Globals.odoName),
//                hardwareMap.get(Limelight3A.class, "limelight"));
        this.odo = new MixedOdo(this);

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
     * 暂停当前线程
     * @param milliseconds 需要暂停的毫秒数
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * 暂停当前线程
     * @param milliseconds 需要暂停的毫秒数
     * @return 可以链式调用
     */
    public Robot waitFor(Long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return this;
    }

    /**
     * 建议将所有需要在主线程每轮循环执行一次的操作都放在这里
     * 应当在opMode每次循环最开始执行
     */
    public void updateAllUnasync() {
        // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
        gamepad1.update();
        odo.update();
        telemetry.update();
    }
}
