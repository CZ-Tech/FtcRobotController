package org.firstinspires.ftc.teamcode.common.interfaces;

/**
 * WPILib 风格的 Command 接口 —— 完整的生命周期模型。
 * <p>
 * 继承自 {@link StoppableTask}，扩展为三段式生命周期：
 * <pre>{@code
 *                          ┌─ isFinished() == true ──┐
 *     init() ──▶  execute() ──▶  end(false)
 *     (一次)      (循环)       (正常结束)
 *                       │
 *                       └── stop() ──▶ end(true)
 *                                     (被中断)
 * }</pre>
 * <p>
 * 与 WPILib {@code Command} 的对应关系：
 * <ul>
 *   <li>{@link #init()} ↔ {@code Command.initialize()}</li>
 *   <li>{@link #execute()} ↔ {@code Command.execute()}</li>
 *   <li>{@link #end(boolean)} ↔ {@code Command.end(boolean interrupted)}</li>
 *   <li>{@link #isFinished()} ↔ {@code Command.isFinished()}</li>
 * </ul>
 * <p>
 * 典型实现：{@code TaskLoopFrame}（后台线程循环）、{@code solutions/} 中的赛季策略。
 *
 * @see StoppableTask
 */
public interface Command extends StoppableTask {

    /**
     * 初始化 —— 命令开始前调用一次。
     * <p>
     * 对应 WPILib {@code Command.initialize()}。
     * 在此处执行一次性资源申请、状态复位或参数计算。
     */
    default void init() {}

    /**
     * 周期性执行 —— 每个调度周期调用一次。
     * <p>
     * 对应 WPILib {@code Command.execute()}。
     * 命令的主要工作逻辑放在此处。
     */
    default void execute() {}

    /**
     * 结束回调 —— 命令终止时调用一次。
     * <p>
     * 对应 WPILib {@code Command.end(boolean interrupted)}。
     * 在此处释放资源、停止线程、关闭硬件。
     *
     * @param interrupted {@code true} 表示被外部中断而非正常完成
     */
    default void end(boolean interrupted) {}

    /**
     * 命令是否已完成工作。
     * <p>
     * 对应 WPILib {@code Command.isFinished()}。
     * 调度器在每次 {@link #execute()} 后检查此方法，
     * 返回 {@code true} 时自动调用 {@link #end(boolean) end(false)}。
     *
     * @return {@code true} 当命令已正常完成
     */
    default boolean isFinished() {
        return false;
    }

    /**
     * 停止命令 —— 以"被中断"方式结束。
     * <p>
     * 等价于调用 {@link #end(boolean) end(true)}。
     * 覆盖 {@link StoppableTask#stop()} 以注入生命周期回调。
     * 必须幂等 —— 多次调用只执行一次 {@code end(true)}。
     */
    @Override
    default void stop() {
        end(true);
    }
}