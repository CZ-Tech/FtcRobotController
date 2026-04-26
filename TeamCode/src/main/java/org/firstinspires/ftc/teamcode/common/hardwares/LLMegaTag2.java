package org.firstinspires.ftc.teamcode.common.hardwares;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.MixedTelemetry;

public class LLMegaTag2 {
    private Limelight3A limelight;
    private MixedTelemetry telemetry;

    public LLMegaTag2(MixedTelemetry telemetry, Limelight3A ll) {
        this.telemetry = telemetry;
        limelight = ll;
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // Switch to pipeline number 0 （目前0应当设为Apriltag）
    }


    /**
     * @param robotYawDegrees 当前机器人的朝向角（必须是角度 Degrees）
     */
    public double[] getPos_MT2(double robotYawDegrees) {
        limelight.updateRobotOrientation(robotYawDegrees);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // 使用 MegaTag2 算法
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double h = botpose.getOrientation().getYaw();
                telemetry.addData("LL Location:", "(" + x + ", " + y + ", " + h + ")");
                return new double[] {x, y};
            }
        }
        return null;
    }

    public double[] getPos() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // 使用 MegaTag2 算法
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("LL Location:", "(" + x + ", " + y + ")");
                return new double[] {x, y};
            }
        }
        return null;
    }

    /**
     * 提供给异步初始化任务直接读取原始帧，不注入陀螺仪数据
     */
    public LLResult getRawResult() {
        return limelight.getLatestResult();
    }
}
