package org.firstinspires.ftc.teamcode.common.interfaces;

/**
 * 可停止任务接口 —— 极简生命周期。
 * <p>
 * 仅提供停止与运行态查询能力，适合只需要资源清理的组件。
 * <p>
 * 对于需要完整生命周期（初始化-周期性执行-结束回调）的组件，请参见 {@link Command}。
 * <p>
 * 迁移说明：原位于 {@code common/frames/StoppableTask.java}，扩展后拆解为
 * {@code StoppableTask}（本接口）与 {@code Command}（全生命周期接口）。
 */
public interface StoppableTask {

    /**
     * 停止任务并释放所有持有资源。
     * <p>
     * 必须幂等 —— 多次调用不应产生副作用。
     */
    void stop();

    /**
     * 任务是否正在运行。
     *
     * @return {@code true} 当任务处于活动状态
     */
    default boolean isRunning() {
        return false;
    }
}