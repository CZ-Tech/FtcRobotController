package org.firstinspires.ftc.teamcode.solutions.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.interfaces.IVisionProvider;
import org.firstinspires.ftc.teamcode.common.util.MixedTelemetry;

public class LLMegaTag2 implements IVisionProvider {
    private Limelight3A limelight;
    private MixedTelemetry telemetry;
    private boolean running = false;

    public LLMegaTag2(MixedTelemetry telemetry, Limelight3A ll) {
        this.telemetry = telemetry;
        limelight = ll;
        limelight.setPollRateHz(100); // 每秒轮询 100 次
        limelight.pipelineSwitch(0);   // 切换至 pipeline 0（AprilTag）
        limelight.start();
        running = true;
    }

    // ==========================================
    // IVisionProvider 实现
    // ==========================================

    @Override
    @Nullable
    public double[] getFieldPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return new double[]{
                        botpose.getPosition().x,           // 米
                        botpose.getPosition().y,           // 米
                        botpose.getOrientation().getYaw()  // 弧度
                };
            }
        }
        return null;
    }

    @Override
    public boolean isReady() {
        return limelight.getStatus() != null && limelight.getStatus().getFps() > 0;
    }

    @Override
    public boolean hasDetection() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    @Override
    public void close() {
        stop();
    }

    @Override
    public void stop() {
        running = false;
        limelight.stop();
    }

    @Override
    public boolean isRunning() {
        return running;
    }

    // ==========================================
    // Limelight 专属方法
    // ==========================================

    /**
     * MegaTag2 定位（需要机器人朝向输入以提高精度）。
     *
     * @param robotYawDegrees 当前机器人的朝向角（角度制）
     */
    @Nullable
    public double[] getPos_MT2(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double h = botpose.getOrientation().getYaw();
                telemetry.addData("LL Location:", "(" + x + ", " + y + ", " + h + ")");
                return new double[]{x, y, h};
            }
        }
        return null;
    }

    /**
     * 标准定位（无需朝向输入）。
     */
    @Nullable
    public double[] getPos() {
        return getFieldPose();
    }

    /**
     * 提供给异步初始化任务直接读取原始帧，不注入陀螺仪数据。
     */
    @Nullable
    public LLResult getRawResult() {
        return limelight.getLatestResult();
    }
}
