package org.firstinspires.ftc.teamcode.common.drivetrain;

import static org.firstinspires.ftc.teamcode.common.Globals.UseVisionLocate;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.frames.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.hardwares.WebCamAprilTagLocate;
import org.firstinspires.ftc.teamcode.common.util.Alliance;

/**
 * 混合里程计系统 (纯偏移量架构)
 * 绝对不调用硬件的 resetPosAndIMU，完全通过 2D 刚体变换实现视觉定位与跨 OpMode 记忆。
 */
public class MixedOdo {
    Robot robot;
    // ==========================================
    // 跨 OpMode 全局静态偏移量记忆
    // ==========================================
    public static boolean isPoseInitialized = false;
    public static double offsetX = 0.0;
    public static double offsetY = 0.0;
    public static double offsetHeading = 0.0;

    private final GoBildaPinpointDriver pinpoint;
    private WebCamAprilTagLocate visualLocater;

    private VisionInitTask initTask;
    private Pose2D currentFieldPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0);

    private Alliance alliance;

    public MixedOdo(Robot robot) {
        this.robot = robot;
        this.pinpoint = robot.hardwareMap.get(GoBildaPinpointDriver.class, Globals.odoName);
        this.alliance = robot.teamColor;

        pinpoint.setOffsets(Globals.odoXOffset, Globals.odoYOffset, DistanceUnit.MM);
        pinpoint.setEncoderResolution(Globals.odoType);
        pinpoint.setEncoderDirections(Globals.odoXDirection, Globals.odoYDirection);

        // 只有机器彻底断电重启后的第一次运行，才允许重置硬件。
        // TeleOp 时不要调用硬件重置，让 Pinpoint 顺着 Auto 继续算。
        if (!isPoseInitialized) {
            pinpoint.resetPosAndIMU();
        }
        if (UseVisionLocate) {
            this.visualLocater = new WebCamAprilTagLocate(robot.hardwareMap.get(WebcamName.class, Globals.logiC270));
        } else {
            isPoseInitialized = true;
        }
        pinpoint.update();

        if (!isPoseInitialized) {
            // 启动异步初始化逻辑 (仅在 Auto 阶段)
            initTask = new VisionInitTask(50, "VisionInitTask");
            initTask.start();
        } else {
            Log.i("MixedOdo", "TeleOp Mode: Restored coordinate offsets from Auto.");
            updateFieldPose(); // 根据缓存的偏移量计算出当前位置
        }
    }

    public void reCollaborate() {
        if (UseVisionLocate) {
            initTask = new VisionInitTask(50, "VisionInitTask");
            initTask.start();
        } else {
            resetPosAndIMU();
        }
    }

    public void resetPosAndIMU() {
        setGlobalPose(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0));
    }

    /**
     * 高频主循环
     */
    public synchronized void update() {
        pinpoint.update();
        if (isPoseInitialized) {
            updateFieldPose();
        }

    }

    public Pose2D getPosition() {
        if (!isPoseInitialized) {
            // 如果还没初始化完，先返回原始里程计坐标（防止空指针或失控）
            return pinpoint.getPosition();
        }
        return currentFieldPose;
    }

    public double getHeading(AngleUnit unit) {
        return getPosition().getHeading(unit);
    }

    /**
     * 核心数学：计算当前的真实场地坐标
     */
    private void updateFieldPose() {
        Pose2D rawOdom = pinpoint.getPosition();
        double rawX = rawOdom.getX(DistanceUnit.METER);
        double rawY = rawOdom.getY(DistanceUnit.METER);
        double rawHeading = rawOdom.getHeading(AngleUnit.RADIANS);

        // 1. 计算全局朝向
        double fieldHeading = rawHeading + offsetHeading;
        while (fieldHeading > Math.PI) fieldHeading -= 2 * Math.PI;
        while (fieldHeading < -Math.PI) fieldHeading += 2 * Math.PI;

        // 2. 将原始 X/Y 向量按朝向偏移量旋转
        double cos = Math.cos(offsetHeading);
        double sin = Math.sin(offsetHeading);
        double rotX = rawX * cos - rawY * sin;
        double rotY = rawX * sin + rawY * cos;

        // 3. 加上平移偏移量
        double fieldX = rotX + offsetX;
        double fieldY = rotY + offsetY;

        currentFieldPose = new Pose2D(DistanceUnit.METER, fieldX, fieldY, AngleUnit.RADIANS, fieldHeading);
    }

    /**
     * 核心数学：根据视觉目标坐标，逆推并锁定全局偏移量 (Offset)
     */
    public synchronized void setGlobalPose(Pose2D targetFieldPose) {
        Pose2D rawOdom = pinpoint.getPosition();
        double rawX = rawOdom.getX(DistanceUnit.METER);
        double rawY = rawOdom.getY(DistanceUnit.METER);
        double rawHeading = rawOdom.getHeading(AngleUnit.RADIANS);

        // 计算角度差
        offsetHeading = targetFieldPose.getHeading(AngleUnit.RADIANS) - rawHeading;

        // 计算旋转后的里程计向量
        double cos = Math.cos(offsetHeading);
        double sin = Math.sin(offsetHeading);
        double rotX = rawX * cos - rawY * sin;
        double rotY = rawX * sin + rawY * cos;

        // 计算平移偏移量：Offset = Target - RotatedOdom
        offsetX = targetFieldPose.getX(DistanceUnit.METER) - rotX;
        offsetY = targetFieldPose.getY(DistanceUnit.METER) - rotY;

        updateFieldPose();
    }

    // ==========================================
    // 异步视觉初始化任务 (基于内置 IMU 角速度防抖版)
    // ==========================================
    private class VisionInitTask extends TaskLoopFrame {
        private static final int TARGET_FRAMES = 50;

        // 大于2mm/s视为在移动
        private static final double MOVEMENT_THRESHOLD_MM_PER_SEC = 2.0;
        // 角速度阈值：大于 1 度/秒 认为被移动 (静止状态下通常是 0.0x 级别的底噪)
        private static final double MOVEMENT_THRESHOLD_DEG_PER_SEC = 2.0;

        private int framesCollected = 0;
        private double sumX = 0, sumY = 0;
        private double sumSin = 0, sumCos = 0;

        public VisionInitTask(int targetUpdateFrequency, String threadName) {
            super(targetUpdateFrequency, threadName);
        }

        @Override
        protected void update() {
            if (framesCollected >= TARGET_FRAMES) {
                double avgX = sumX / framesCollected;
                double avgY = sumY / framesCollected;
                double avgHeadingRad = Math.atan2(sumSin / framesCollected, sumCos / framesCollected);

                Pose2D optimizedInitPose = new Pose2D(DistanceUnit.METER, avgX, avgY, AngleUnit.RADIANS, avgHeadingRad);

                // 锁定偏移量
                setGlobalPose(optimizedInitPose);
                Log.i(threadName, "Initialization Complete! Global Pose locked at: " + optimizedInitPose.toString() + ", " + optimizedInitPose.getHeading(AngleUnit.DEGREES) + "DEG");

                stop();
                isPoseInitialized = true;
                return;
            }

            robot.telemetry.addData("Odo Visual Initializing", "got "+framesCollected+"/"+TARGET_FRAMES);

            double rates = pinpoint.getVelX(DistanceUnit.MM) + pinpoint.getVelY(DistanceUnit.MM);

            // 只要 X(Roll), Y(Pitch), Z(Yaw) 任意一轴的旋转速率超过阈值，说明被碰了
            boolean isMoving = Math.abs(pinpoint.getVelX(DistanceUnit.MM)) > MOVEMENT_THRESHOLD_MM_PER_SEC ||
                    Math.abs(pinpoint.getVelY(DistanceUnit.MM)) > MOVEMENT_THRESHOLD_MM_PER_SEC ||
                    Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)) > MOVEMENT_THRESHOLD_DEG_PER_SEC;

            if (isMoving) {
                // 如果检测到移动，清空所有视觉累加数据，重新开始
                if (framesCollected > 0) {
                    framesCollected = 0;
                    sumX = 0; sumY = 0;
                    sumSin = 0; sumCos = 0;
                    Log.d(threadName, "Bump detected by Hub IMU! Restarting vision collection.");
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                // 如果正在移动，本帧直接 return，不要把运动中模糊的 Limelight 画面加进去
                return;
            }

            // 读取数据进行累加
            Pose2D result = visualLocater.getPos();
            if (result != null) {
                sumX += result.getX(DistanceUnit.METER);
                sumY += result.getY(DistanceUnit.METER);

                double yawRad = result.getHeading(AngleUnit.RADIANS);
                sumSin += Math.sin(yawRad);
                sumCos += Math.cos(yawRad);

                framesCollected++;
            }
        }
    }


    /**
     * 计算从球门到当前坐标点的向量方向角度。
     * 规定 x轴正方向为 0°，逆时针方向角度递增。
     *
     * @param currentX 当前点的X坐标
     * @param currentY 当前点的Y坐标
     * @return 角度值，范围在 [0.0, 360.0) 之间
     */
    public double calculateDirection(double currentX, double currentY, DistanceUnit unit) {
        // 计算两点在X和Y轴上的差值（方向：当前点 - 定点）
        double dx = currentX - alliance.getGoalPos().getX(unit);
        double dy = currentY - alliance.getGoalPos().getY(unit);

        // 使用 atan2 计算弧度
        // Math.atan2 返回的值范围是 [-π, π]
        double radians = Math.atan2(dy, dx);

        // 将弧度转换为角度，此时范围是 [-180°, 180°]
        double degrees = Math.toDegrees(radians);

        // 将角度归一化到 [0°, 360°) 之间
        // 如果你需要保持 -180 到 180 度的范围，可以直接 return degrees;
        degrees = (degrees + 360.0) % 360.0;

        return degrees;
    }
}