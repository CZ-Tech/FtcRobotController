package org.firstinspires.ftc.teamcode.common.interfaces;

import androidx.annotation.NonNull;

/**
 * 轨迹跟踪统一 API —— 对标 WPILib 的 {@code Trajectory} / {@code RamseteCommand} 模式。
 * <p>
 * 定义轨迹的运行时行为，不关心轨迹的具体生成方式。
 * <p>
 * 典型实现：
 * <ul>
 *   <li>{@code PinpointTrajectory} —— 时序驱动的三次样条轨迹（时间轴）</li>
 *   <li>{@code SplineTracker} —— 位置驱动的样条跟踪（牛顿投影，空间轴）</li>
 * </ul>
 * <p>
 * 使用模式：
 * <pre>{@code
 * ITrajectory trajectory = ...;
 * trajectory.startMove();
 * while (opModeIsActive && !trajectory.isFinished()) {
 *     trajectory.update(robot);
 *     robot.updateAllUnasync();
 * }
 * }</pre>
 */
public interface ITracker extends StoppableTask {

    /**
     * 开始轨迹执行。初始化内部状态与计时器。
     */
    void startMove();

    /**
     * 更新轨迹跟踪。应在每个控制循环中调用。
     *
     * @return {@code true} 当轨迹已完成，{@code false} 当仍处于运动中
     */
    boolean update();

    /**
     * 轨迹是否已完成。
     *
     * @return {@code true} 当轨迹已到达终点
     */
    boolean isFinished();

    /**
     * 强制停止轨迹跟踪并复位内部状态。
     */
    void stop();

    /**
     * 获取当前轨迹进度。
     *
     * @return 进度值 [0.0, 1.0]，1.0 表示已完成
     */
    double getProgress();

    /**
     * 获取当前轨迹段的目标位置（用于调试/Telemetry）。
     *
     * @return 长度为 3 的数组 [x, y, heading]，单位分别为 [米, 米, 弧度]
     */
    @NonNull
    double[] getTargetPose();
}
