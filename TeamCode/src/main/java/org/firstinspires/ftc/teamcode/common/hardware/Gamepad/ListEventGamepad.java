package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * 一个事件驱动的 Gamepad 包装器，它提供了一个流畅的 API，用于将监听器附加到
 * 按键的按下、释放和持续按住等事件上。
 *
 * 此类以固定的时间间隔轮询手柄状态，并将事件分发给已注册的监听器。
 * 它支持同步和异步执行操作。
 *
 * 用法:
 * <pre>{@code
 * ListEventGamepad eventGamepad = new ListEventGamepad();
 *
 * // 示例：当 'A' 键第一次被按下时，打印一条消息。
 * eventGamepad.button(Buttons.A)
 *             .onPress(() -> System.out.println("A 键被按下了!"));
 *
 * // 示例：当 'X' 键被按住时，启动一个马达。
 * eventGamepad.button(Buttons.X)
 *             .whenDown(pressCount -> myMotor.setPower(0.5));
 *
 * // 示例：当 'X' 键被释放时，停止马达。
 * eventGamepad.button(Buttons.X)
 *             .onRelease(() -> myMotor.setPower(0));
 *
 * // 示例：异步执行一个操作。
 * eventGamepad.button(Buttons.B)
 *             .onPressAsync(() -> {
 *                 // 这段代码将在一个后台线程中运行。
 *             });
 * }</pre>
 */
public class ListEventGamepad extends Gamepad {

    private Executor executor = null;
    private final ScheduledExecutorService pollingExecutor =
            Executors.newSingleThreadScheduledExecutor();

    // 一个 Map，用于为每个被监视的按钮存储和复用其专用的事件处理器。
    private final Map<Buttons, ButtonEventHandler> buttonHandlers = new ConcurrentHashMap<>();

    public ListEventGamepad() {
        // 以稍高的频率进行轮询，以获得更好的响应性。
        pollingExecutor.scheduleWithFixedDelay(this::pollControllerState, 0, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * 主轮询循环。它会遍历所有注册的按钮处理器并触发它们各自的轮询逻辑。
     */
    public void pollControllerState() {
        for (ButtonEventHandler handler : buttonHandlers.values()) {
            handler.poll(this);
        }
    }

    /**
     * 获取或创建一个特定按钮的处理器，为事件注册提供一个流畅的接口。
     *
     * @param button 要监视的按钮。
     * @return 用于链式注册事件的 {@link ButtonEventHandler}。
     */
    public ButtonEventHandler listen(Buttons button) {
        // computeIfAbsent 确保了每个按钮只有一个处理器实例。
        return buttonHandlers.computeIfAbsent(button, k -> new ButtonEventHandler());
    }

    private synchronized Executor getExecutor() {
        if (executor == null) {
            // 使用一个固定大小的线程池以更好地管理资源，并防止无限制地创建线程。
            executor = Executors.newFixedThreadPool(5);
        }
        return executor;
    }

    /**
     * 一个专门用于管理单个按钮的状态和监听器的处理器。
     * 此类将与单个按钮相关的所有逻辑封装在一起，使系统更加模块化。
     */
    public class ButtonEventHandler {

        // --- 事件监听器列表 ---
        private final CopyOnWriteArrayList<Runnable> onPressActions = new CopyOnWriteArrayList<>();
        private final CopyOnWriteArrayList<Runnable> onReleaseActions = new CopyOnWriteArrayList<>();
        private final CopyOnWriteArrayList<Consumer<Integer>> onToggleActions = new CopyOnWriteArrayList<>();
        private final CopyOnWriteArrayList<Consumer<Integer>> whenDownActions = new CopyOnWriteArrayList<>();

        // --- 状态追踪 ---
        private int pressCount = 0;

        /**
         * 轮询此处理器负责的特定按钮的状态。
         * @param gamepad 要读取状态的游戏手柄。
         */
        void poll(Gamepad gamepad) {
            Buttons button = buttonHandlers.entrySet().stream()
                    .filter(entry -> entry.getValue() == this)
                    .map(Map.Entry::getKey)
                    .findFirst()
                    .orElse(null);

            if (button == null) return; // 理论上不会发生

            // 处理“刚刚按下”的事件
            if (button.wasJustPressed(gamepad)) {
                pressCount++;
                for (Runnable action : onPressActions) action.run();
                for (Consumer<Integer> action : onToggleActions) action.accept(pressCount);
            }

            // 处理“刚刚释放”的事件
            if (button.wasJustReleased(gamepad)) {
                for (Runnable action : onReleaseActions) action.run();
            }

            // 处理“持续按下”的事件
            if (button.ifPressed(gamepad)) {
                for (Consumer<Integer> action : whenDownActions) action.accept(pressCount);
            }
        }

        public ButtonEventHandler onPress(Runnable action) {
            onPressActions.add(action);
            return this;
        }

        public ButtonEventHandler onPress(Consumer<Integer> action) {
            onToggleActions.add(action);
            return this;
        }

        public ButtonEventHandler onPressAsync(Runnable action) {
            return onPress(new AsyncRunnable(action, getExecutor()));
        }

        public ButtonEventHandler onPressAsync(Consumer<Integer> action) {
            return onPress(new AsyncConsumer<>(action, getExecutor()));
        }

        public ButtonEventHandler onRelease(Runnable action) {
            onReleaseActions.add(action);
            return this;
        }

        public ButtonEventHandler onReleaseAsync(Runnable action) {
            return onRelease(new AsyncRunnable(action, getExecutor()));
        }



        public ButtonEventHandler whenDown(Consumer<Integer> action) {
            whenDownActions.add(action);
            return this;
        }

        public ButtonEventHandler whenDownAsync(Consumer<Integer> action) {
            return whenDown(new AsyncConsumer<>((pressedTimes) -> {
                while ()
                action.accept(pressedTimes);
            }, getExecutor()));
        }
    }

    // --- 用于异步操作的辅助类 ---
    private static class AsyncRunnable implements Runnable {
        private final Runnable action;
        private final Executor executor;

        public AsyncRunnable(Runnable action, Executor executor) {
            this.action = action;
            this.executor = executor;
        }

        @Override
        public void run() {
            executor.execute(action);
        }
    }

    private static class AsyncConsumer<T> implements Consumer<T> {
        private final Consumer<T> action;
        private final Executor executor;

        public AsyncConsumer(Consumer<T> action, Executor executor) {
            this.action = action;
            this.executor = executor;
        }

        @Override
        public void accept(T arg) {
            executor.execute(() -> action.accept(arg));
        }
    }
}
