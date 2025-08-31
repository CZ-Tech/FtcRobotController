package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.LinearReturns;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.PositionReturns;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
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
 * eventGamepad.listen(Buttons.A)
 *             .onPress(() -> System.out.println("A 键被按下了!"));
 *
 * // 示例：当 'X' 键被按住时，启动一个马达。
 * eventGamepad.listen(Buttons.X)
 *             .whenDown(pressCount -> myMotor.setPower(0.5));
 *
 * // 示例：当 'X' 键被释放时，停止马达。
 * eventGamepad.listen(Buttons.X)
 *             .onRelease(() -> myMotor.setPower(0));
 *
 * // 示例：异步执行一个操作。
 * eventGamepad.listen(Buttons.B)
 *             .onPressAsync(() -> {
 *                 // 这段代码将在一个后台线程中运行。
 *             });
 * }</pre>
 */
public class ListEventGamepad {
    // 一个 Map，用于为每个被监视的按钮存储和复用其专用的事件处理器。
    private final Map<Buttons, ButtonEventHandler> buttonHandlers = new ConcurrentHashMap<>();
    private final Gamepad gamepad;

    public ListEventGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * 主轮询入口方法。它会遍历所有注册的按钮处理器并触发它们各自的轮询逻辑。
     */
    public void update() {
        for (ButtonEventHandler handler : buttonHandlers.values()) {
            handler.poll(gamepad);
        }
    }

    /**
     * 获取或创建一个特定按钮的处理器，为事件注册提供一个流畅的接口。
     *
     * @param button 要监听的按钮。
     * @return 用于链式注册事件的 {@link ButtonEventHandler}。
     */
    public ButtonEventHandler listen(Buttons button) {
        // computeIfAbsent 确保了每个按钮只有一个处理器实例。
        return buttonHandlers.computeIfAbsent(button, k -> new ButtonEventHandler(k));
    }

    public float getLeaner(LinearReturns trigger) {
        return trigger.getFloatValue(gamepad);
    }

    public float[] getPos(PositionReturns stick) {
        return stick.getPos(gamepad);
    }

    /**
     * 一个专门用于管理单个按钮的状态和监听器的处理器。
     * 此类将与单个按钮相关的所有逻辑封装在一起，使系统更加模块化。
     */
    public class ButtonEventHandler {
        private final Buttons button;

        // --- 事件监听器列表 ---
        private final CopyOnWriteArrayList<Runnable> onPressActions = new CopyOnWriteArrayList<>();
        private final CopyOnWriteArrayList<Runnable> onReleaseActions = new CopyOnWriteArrayList<>();
        private final CopyOnWriteArrayList<Consumer<Integer>> onToggleActions = new CopyOnWriteArrayList<>();

        // --- 状态追踪 ---
        private int pressCount = 0;

        public ButtonEventHandler(Buttons button) {
            this.button = button;
        }

        /**
         * 轮询此处理器负责的特定按钮的状态。
         * @param gamepad 要读取状态的游戏手柄。
         */
        void poll(Gamepad gamepad) {
            if (button == null) return;

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
        }

        public ButtonEventHandler onPress(Runnable action) {
            onPressActions.add(action);
            return this;
        }

        public ButtonEventHandler onPress(Consumer<Integer> action) {
            onToggleActions.add(action);
            return this;
        }


        public ButtonEventHandler onRelease(Runnable action) {
            onReleaseActions.add(action);
            return this;
        }


    }
}
