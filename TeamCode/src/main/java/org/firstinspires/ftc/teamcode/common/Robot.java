package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.drivetrain.MixedOdo;
import org.firstinspires.ftc.teamcode.common.drivetrain.OdoDrivetrain;
import org.firstinspires.ftc.teamcode.common.Gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.common.Gamepad.GamepadReflectBinder;
import org.firstinspires.ftc.teamcode.solutions.hardware.WebCamAprilTag;
import org.firstinspires.ftc.teamcode.common.interfaces.IVisionProvider;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.MixedTelemetry;
import org.firstinspires.ftc.teamcode.solutions.chassis.MecanumChassis;

public class Robot {
    public HardwareMap hardwareMap;
    public MixedTelemetry telemetry;
    public LinearOpMode opMode;
    public OdoDrivetrain odoDrivetrain;
    public GamepadEx gamepad1;
    public GamepadEx gamepad2;
    public Alliance teamColor;
    public MixedOdo odo;


    /**
     * 初始化机器人
     *
     * @param opMode 在 opMode 里通过 robot.init(this); 进行初始化。
     */
    public void init(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap; //硬件映射
        this.telemetry = new MixedTelemetry(opMode.telemetry);//DH上的遥测

        // 视觉硬件（可通过 Globals.UseVisionLocate 开关）
        IVisionProvider vision = null;
        if (Globals.UseVisionLocate) {
            vision = new WebCamAprilTag(hardwareMap.get(WebcamName.class, Globals.logiC270));
        }

        this.odo = new MixedOdo(this, vision);

        this.gamepad1 = new GamepadEx(opMode.gamepad1, opMode::opModeIsActive); //一个控制器
        this.gamepad2 = new GamepadEx(opMode.gamepad2, opMode::opModeIsActive); //另一个控制器

        // 电机初始化
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, Globals.LeftFrontMotor);
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, Globals.RightFrontMotor);
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, Globals.LeftBackMotor);
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, Globals.RightBackMotor);

        lf.setDirection(Globals.LeftFrontMotorDirection);
        rf.setDirection(Globals.RightFrontMotoDirection);
        lb.setDirection(Globals.LeftBackMotorDirection);
        rb.setDirection(Globals.RightBackMotorDirection);

        this.odoDrivetrain = new MecanumChassis(lf, rf, lb, rb);
        this.odoDrivetrain.setHeadingSource(() -> odo.getHeadingRadians());

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
     * @param millisecond 需要暂停的毫秒数
     * @return 可以链式调用
     */
    public Robot waitFor(double millisecond) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= millisecond && opMode.opModeIsActive());
        return this;
    }

    /**
     * 并行运行多个任务
     *
     * @param commands 任务列表
     * @return Robot
     */
    public Robot syncRun(Runnable... commands) {
        for (Runnable command : commands) {
            new Thread(command).start();
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
        gamepad2.update();
        odo.update();
        telemetry.update();
    }
}
