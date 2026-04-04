package org.firstinspires.ftc.teamcode.common.hardwares.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers.Buttons;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers.LinearReturns;
import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers.PositionReturns;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class GamepadEx {
    private final Gamepad gamepad;
    // 传入一个函数来判断当前是否处于 Active 状态 (如: opMode::opModeIsActive)
    private final BooleanSupplier isOpModeActive;

    private final Map<String, EventBinding<Runnable>> onPressActions = new ConcurrentHashMap<>();
    private final Map<String, EventBinding<Runnable>> onReleaseActions = new ConcurrentHashMap<>();
    private final Map<String, EventBinding<Consumer<Integer>>> onToggleActions = new ConcurrentHashMap<>();
    private final Map<String, EventBinding<Runnable>> whenDownActions = new ConcurrentHashMap<>();
    private final Map<String, EventBinding<Runnable>> whenUpActions = new ConcurrentHashMap<>();

    private final AtomicInteger anonymousCounter = new AtomicInteger(0);

    // ==========================================
    // 所有的按钮成员变量展开
    // ==========================================
    public final ButtonHandler DPAD_UP = new ButtonHandler(Buttons.DPAD_UP);
    public final ButtonHandler DPAD_DOWN = new ButtonHandler(Buttons.DPAD_DOWN);
    public final ButtonHandler DPAD_LEFT = new ButtonHandler(Buttons.DPAD_LEFT);
    public final ButtonHandler DPAD_RIGHT = new ButtonHandler(Buttons.DPAD_RIGHT);

    public final ButtonHandler A = new ButtonHandler(Buttons.A);
    public final ButtonHandler B = new ButtonHandler(Buttons.B);
    public final ButtonHandler X = new ButtonHandler(Buttons.X);
    public final ButtonHandler Y = new ButtonHandler(Buttons.Y);

    public final ButtonHandler GUIDE = new ButtonHandler(Buttons.GUIDE);
    public final ButtonHandler START = new ButtonHandler(Buttons.START);
    public final ButtonHandler BACK = new ButtonHandler(Buttons.BACK);

    public final ButtonHandler LEFT_BUMPER = new ButtonHandler(Buttons.LEFT_BUMPER);
    public final ButtonHandler RIGHT_BUMPER = new ButtonHandler(Buttons.RIGHT_BUMPER);
    public final ButtonHandler LEFT_STICK_BUTTON = new ButtonHandler(Buttons.LEFT_STICK_BUTTON);
    public final ButtonHandler RIGHT_STICK_BUTTON = new ButtonHandler(Buttons.RIGHT_STICK_BUTTON);

    public final ButtonHandler CIRCLE = new ButtonHandler(Buttons.CIRCLE);
    public final ButtonHandler CROSS = new ButtonHandler(Buttons.CROSS);
    public final ButtonHandler TRIANGLE = new ButtonHandler(Buttons.TRIANGLE);
    public final ButtonHandler SQUARE = new ButtonHandler(Buttons.SQUARE);
    public final ButtonHandler SHARE = new ButtonHandler(Buttons.SHARE);
    public final ButtonHandler OPTIONS = new ButtonHandler(Buttons.OPTIONS);
    public final ButtonHandler PS = new ButtonHandler(Buttons.PS);

    // 用于快速遍历轮询的数组
    private final ButtonHandler[] allHandlers = {
            DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
            A, B, X, Y, GUIDE, START, BACK,
            LEFT_BUMPER, RIGHT_BUMPER, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
            CIRCLE, CROSS, TRIANGLE, SQUARE, SHARE, OPTIONS, PS
    };

    public enum ActionType {
        ON_PRESS,
        ON_RELEASE,
        ON_TOGGLE,
        WHEN_DOWN,
        WHEN_UP
    }

    /**
     * @param gamepad        手柄实例
     * @param isOpModeActive 提供 OpMode 状态的方法引用。例如: `this::opModeIsActive`
     */
    public GamepadEx(Gamepad gamepad, BooleanSupplier isOpModeActive) {
        this.gamepad = gamepad;
        this.isOpModeActive = isOpModeActive;
    }

    public void update() {
        // 统一在这里进行一次性读取，防止破坏性读取导致后续wasJustPressed
        Map<Buttons, Boolean> justPressedCache = new EnumMap<>(Buttons.class);
        Map<Buttons, Boolean> justReleasedCache = new EnumMap<>(Buttons.class);

        for (ButtonHandler handler : allHandlers) {
            justPressedCache.put(handler.button, handler.button.wasJustPressed(gamepad));
            justReleasedCache.put(handler.button, handler.button.wasJustReleased(gamepad));
        }

        for (EventBinding<Runnable> b : onPressActions.values()) {
            if (Boolean.TRUE.equals(justPressedCache.get(b.button))) {
                b.action.run();
            }
        }
        for (EventBinding<Consumer<Integer>> b : onToggleActions.values()) {
            if (Boolean.TRUE.equals(justPressedCache.get(b.button))) {
                b.toggleCount++; // 递增当前独立绑定的计数器;
                b.action.accept(b.toggleCount);
            }
        }
        for (EventBinding<Runnable> b : onReleaseActions.values()) {
            if (Boolean.TRUE.equals(justReleasedCache.get(b.button))) b.action.run();
        }
        for (EventBinding<Runnable> b : whenDownActions.values()) {
            if (b.button.ifPressed(gamepad)) b.action.run();
        }
        for (EventBinding<Runnable> b : whenUpActions.values()) {
            if (!b.button.ifPressed(gamepad)) b.action.run();
        }
    }

    public float get(LinearReturns trigger) { return trigger.getFloatValue(gamepad); }
    public float[] get(PositionReturns stick) { return stick.getPos(gamepad); }

    // ==========================================
    // 轻量级处理器
    // ==========================================
    /**
     * 极轻量级事件分发器。
     * 本身不存储任何状态和集合，仅作为生成联合主键并向外层 5 张大表注册事件的代理。
     */
    public class ButtonHandler {
        private final Buttons button;

        public ButtonHandler(Buttons button) {
            this.button = button;
        }

        /**
         * 状态检查器：防呆防泄漏。
         * 如果在 OpMode 运行期间（Active 状态）尝试绑定匿名 Lambda，直接抛出异常，强制 Fail-fast。
         */
        private void checkStateAndThrow() {
            if (isOpModeActive.getAsBoolean()) {
                throw new IllegalStateException(
                        "Fail-fast: Cannot bind anonymous lambda to [" + button.name() +
                                "] while OpMode is active! This prevents memory leaks in loop(). " +
                                "Use the method with an explicit String ID instead."
                );
            }
        }

        // ==========================================
        // 1. 无 ID 绑定 (Init 阶段专用，Active 抛异常)
        // ==========================================

        public GamepadEx onPress(Runnable action) {
            checkStateAndThrow();
            String key = button.name() + "_ANON_" + anonymousCounter.getAndIncrement();
            onPressActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx onPress(Consumer<Integer> action) {
            checkStateAndThrow();
            String key = button.name() + "_ANON_" + anonymousCounter.getAndIncrement();
            onToggleActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx onRelease(Runnable action) {
            checkStateAndThrow();
            String key = button.name() + "_ANON_" + anonymousCounter.getAndIncrement();
            onReleaseActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx whenDown(Runnable action) {
            checkStateAndThrow();
            String key = button.name() + "_ANON_" + anonymousCounter.getAndIncrement();
            whenDownActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx whenUp(Runnable action) {
            checkStateAndThrow();
            String key = button.name() + "_ANON_" + anonymousCounter.getAndIncrement();
            whenUpActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        // ==========================================
        // 2. 带 ID 绑定 (允许 loop() 运行时安全覆盖)
        // ==========================================

        public GamepadEx onPress(String id, Runnable action) {
            String key = button.name() + "_" + id;
            onPressActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx onPress(String id, Consumer<Integer> action) {
            String key = button.name() + "_" + id;
            onToggleActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx onRelease(String id, Runnable action) {
            String key = button.name() + "_" + id;
            onReleaseActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx whenDown(String id, Runnable action) {
            String key = button.name() + "_" + id;
            whenDownActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }

        public GamepadEx whenUp(String id, Runnable action) {
            String key = button.name() + "_" + id;
            whenUpActions.put(key, new EventBinding<>(button, action));
            return GamepadEx.this;
        }


    }

    public ButtonHandler getHandler(Buttons targetButton) {
        for (GamepadEx.ButtonHandler handler : allHandlers) {
            if (handler.button == targetButton) return handler;
        }
        throw new IllegalArgumentException("未找到对应的按键处理器: " + targetButton.name());
    }

    // 绑定数据包装类
    private static class EventBinding<T> {
        final Buttons button;
        final T action;
        int toggleCount = 0; // 核心改动：专属的状态计数器
        EventBinding(Buttons button, T action) {
            this.button = button;
            this.action = action;
        }
    }

}