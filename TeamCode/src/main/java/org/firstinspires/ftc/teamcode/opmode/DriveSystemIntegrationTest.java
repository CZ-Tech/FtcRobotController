package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.AdaptiveTractionController;
import org.firstinspires.ftc.teamcode.common.drive.AntiLockTractionSystem;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.LinearReturns;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.PositionReturns;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;
import org.firstinspires.ftc.teamcode.tasks.ATCUpdater;
import org.firstinspires.ftc.teamcode.tasks.I2CUpdater;

import java.util.Locale;

/**
 * 这是一个用于 AdaptiveTractionController 和 AntiLockTractionSystem 的集成测试程序。
 *
 * 使用方法:
 * 1. 将机器人放置在有足够开阔空间的场地上。
 * 2. 在 Driver Station 上选择并运行此 OpMode。
 * 3. 按照屏幕上的遥测指示进行操作和观察。
 * 4. 程序会自动按顺序执行一系列测试用例。
 */
@TeleOp(name = "Drive System Integration Test", group = "Debug")
public class DriveSystemIntegrationTest extends LinearOpMode {

    private Robot robot;
    private GoBildaPinpointDataAsync odo;
    private AntiLockTractionSystem ats;
    private AdaptiveTractionController atc;

    // 一个用于在测试之间暂停的辅助方法
    private void waitAndShow(String message, int milliseconds) {
        telemetry.addLine("========================================");
        telemetry.addLine(message);
        telemetry.addLine(String.format(Locale.US, "将在 %d 秒后继续...", milliseconds / 1000));
        telemetry.update();
        sleep(milliseconds);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 初始化 ---
        telemetry.addLine("正在初始化系统，请稍候...");
        telemetry.update();

        // 使用单例模式初始化 Robot
        robot = Robot.getInstance();
        robot.init(this);

        I2CUpdater.getInstance().start();
        ATCUpdater.getInstance().start();

        telemetry.addLine("初始化完成。");
        telemetry.addLine("将机器人放置在开阔平坦的地面上。");
        telemetry.addLine("按下 Start 开始测试...");
        telemetry.update();

        waitForStart();

        testATS_BasicMove();

        // --- 测试结束 ---
        I2CUpdater.getInstance().stop();
        ATCUpdater.getInstance().stop();
        waitAndShow("所有测试已完成。", 10000);
    }


    /**
     * 测试用例 1: 验证 ATS 和 ListEventGamepad 的基本运动控制
     */
    private void testATS_GamepadMove() {
        telemetry.clearAll();
        telemetry.addLine("--- 测试 1: ATS 基础运动 ---");
        telemetry.addLine("--- 请用手柄输入 ---");
        telemetry.update();
        while (opModeIsActive()) {
            float[] leftStick = Robot.getInstance().gamepad1.getPos(PositionReturns.LEFT_STICK);
            float[] rightStick = Robot.getInstance().gamepad1.getPos(PositionReturns.RIGHT_STICK);
            ats.setTargetPower(leftStick[0], leftStick[1], rightStick[0]);
        }

    }

    /**
     * 测试用例 0: 验证 ATS 的基本运动控制
     */
    private void testATS_BasicMove() {
        telemetry.clearAll();
        telemetry.addLine("--- 测试 1: ATS 基础运动 ---");
        telemetry.addLine("--- 准备前进1s ---");
        telemetry.update();
        ats.setTargetPower(0, 0.5, 0);
        sleep(1000);
        ats.setTargetPower(0,0,0);
        telemetry.addLine("--- 准备左前移动0.5s ---");
        telemetry.update();
        sleep(1000);
        ats.setTargetPower(0.5,0.5,0);
    }

    /**
     * 测试用例 2: 验证 ATC 的摩擦力自适应学习
     */
    private void testATC_FrictionLearning() {
        waitAndShow("--- 测试 2: ATC 摩擦力学习 ---\n将测试全功率直线加速以触发学习。", 5000);

        Robot.getInstance().gamepad1.listen(Buttons.B).onPress(() -> {
            atc.setRunAntiLock();
            ats.setTargetPower(0, 0, 0);
        });

        atc.resetLearning();
        double initialAccel = atc.getLearnedMaxAcceleration(); // 需要一个 getter 方法

        telemetry.addData("初始学习加速度 (mm/s^2)", "%.2f", initialAccel);
        telemetry.addLine("准备全功率前进 1 秒后刹车...您可以随时按B以停止电机动作");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        ats.setTargetPower(1.0, 0, 0); // 直接使用ATS施加最大动力

        while(opModeIsActive() && timer.seconds() < 1) {
            telemetry.addLine("正在加速并学习...");
            telemetry.addData("当前学习加速度 (mm/s^2)", "%.2f", atc.getLearnedMaxAcceleration());
            telemetry.addData("ATS 是否在极限摩擦下工作", ats.isUsingLimitingFriction(3));
            telemetry.addData("当前速度 (mm/s)", "%.2f", Math.hypot(odo.getVelX(), odo.getVelY()));
            telemetry.update();
        }

        atc.autoBrake();
        while(opModeIsActive() && timer.seconds() < 1) {
            telemetry.addLine("正在刹车并学习...");
            telemetry.addData("当前学习加速度 (mm/s^2)", "%.2f", atc.getLearnedMaxAcceleration());
            telemetry.addData("ATS 是否在极限摩擦下工作", ats.isUsingLimitingFriction(3));
            telemetry.addData("当前速度 (mm/s)", "%.2f", Math.hypot(odo.getVelX(), odo.getVelY()));
            telemetry.update();
        }

        ats.setTargetPower(0, 0, 0);
        ats.update();

        double finalAccel = atc.getLearnedMaxAcceleration();

        telemetry.addLine("学习测试完成。");
        telemetry.addData("初始值", "%.2f", initialAccel);
        telemetry.addData("最终值", "%.2f", finalAccel);
//        if (Math.abs(finalAccel - initialAccel) > 100) {
//            telemetry.addLine("结果: 成功! 加速度值已更新。");
//        } else {
//            telemetry.addLine("结果: 失败或无明显变化。请确保地面有足够摩擦力。");
//        }
        telemetry.update();
        sleep(5000);
    }

    /**
     * 测试用例 3: 验证 ATC 的点到点停止功能
     */
    private void testATC_RunToPosition_Stop() {
        waitAndShow("--- 测试 3: ATC 点到点停止 ---\n机器人将自动行驶到 X=1000mm, Y=0mm。", 5000);

        // 设置目标和回调
        final boolean[] targetReached = {false};
        atc.setTargetPos(1000, 0, 0, () -> {
            targetReached[0] = true;
            atc.setRunAntiLock(); // 切换回手动模式
            telemetry.log().add("目标已到达! 回调已执行。");
        });
        atc.setSpeedOnTargetPos(0, 0); // 确保目标是停止
        atc.setRunToPos();

        while(opModeIsActive() && !targetReached[0]) {
            telemetry.addLine("正在执行 RUN_TO_POSITION...");
            telemetry.addData("当前位置 X", "%.1f", odo.getPosX());
            telemetry.addData("当前位置 Y", "%.1f", odo.getPosY());
            telemetry.addData("当前航向 (deg)", "%.1f", Math.toDegrees(odo.getHeading()));
            telemetry.addData("目标 X", "1000.0");
            telemetry.addData("距目标距离 (mm)", "%.1f", Math.hypot(1000 - odo.getPosX(), 0 - odo.getPosY()));
            telemetry.update();
        }

        waitAndShow("点到点停止测试完成。", 2000);
        telemetry.addLine("最终位置:");
        telemetry.addData("X (mm)", "%.1f", odo.getPosX());
        telemetry.addData("Y (mm)", "%.1f", odo.getPosY());
        telemetry.addData("误差 X (mm)", "%.1f", odo.getPosX() - 1000);
        telemetry.addData("误差 Y (mm)", "%.1f", odo.getPosY() - 0);
        telemetry.update();
        sleep(5000);
    }

}