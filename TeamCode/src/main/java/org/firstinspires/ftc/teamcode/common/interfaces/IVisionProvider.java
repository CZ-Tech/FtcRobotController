package org.firstinspires.ftc.teamcode.common.interfaces;

import androidx.annotation.Nullable;

/**
 * 视觉定位接口
 * <p>
 * 提供从视觉系统获取场地坐标系下
 * 机器人位姿的统一方式。
 * <p>
 * 单位：
 * <ul>
 *   <li>位置：米（meter）</li>
 *   <li>朝向：弧度（radian）</li>
 * </ul>
 * <p>
 * 参考实现：{@code solutions/hardware/C270AprilTag.java}, {@code Limelight3A.java}
 */
public interface IVisionProvider extends StoppableTask {

    /**
     * 获取视觉系统估计的场地坐标位姿。
     *
     * @return 位姿信息 [X, Y, Heading] (m, rad)，若当前无有效检测则返回 {@code null}
     */
    @Nullable
    double[] getFieldPose();

    /**
     * 视觉系统是否已就绪并可用。
     *
     * @return {@code true} 当视觉处理器已完成初始化并可提供检测
     */
    boolean isReady();

    /**
     * 获取视觉系统是否在当前帧检测到了有效目标。
     *
     * @return {@code true} 当在当前帧中有有效检测
     */
    boolean hasDetection();

    /**
     * 释放视觉系统持有的资源（摄像头、处理器等）。
     * 应等效于 {@link #stop()}，但语义更明确。
     */
    void close();
}
