package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

public class ListEventGamepad extends Gamepad {
    public static boolean PRESS = true;
    public static boolean RELEASE = false;

    private Executor executor = null;

    private final ScheduledExecutorService pollingExecutor =
            Executors.newSingleThreadScheduledExecutor();
    private final Map<Buttons, CopyOnWriteArrayList<Runnable>>
            registeredButtonPress = initEventMap(new HashMap<>());

    private final Map<Buttons, CopyOnWriteArrayList<Runnable>>
            registeredButtonRelease = initEventMap(new HashMap<>());

    private final Map<Buttons, CopyOnWriteArrayList<Consumer<Integer>>>
            registeredButtonToggle = initEventMap(new HashMap<>());

    private final ConcurrentHashMap<Buttons, Integer> buttonState = new ConcurrentHashMap<>();

    private static <T> Map<Buttons, CopyOnWriteArrayList<T>>
    initEventMap(Map<Buttons, CopyOnWriteArrayList<T>> mapToInit) {
        for (Buttons button : Buttons.values()) {
            mapToInit.put(button, new CopyOnWriteArrayList<>());
        }
        return Collections.unmodifiableMap(mapToInit);
    }

    private synchronized Executor getexecutor() {
        if (executor == null) {
            executor = Executors.newCachedThreadPool();
        }
        return executor;
    }

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

    public ListEventGamepad() {
        pollingExecutor.schedule(this::pollControllerState, 10L, TimeUnit.MILLISECONDS);
    }

    public void pollControllerState() { // This method is not public, so it remains void
        for (Buttons button : Buttons.values()) {
            if (button.wasJustPressed(this)) {
                for (Runnable action :
                        Objects.requireNonNull(registeredButtonPress.get(button))) {
                    action.run();
                }
                // 记录按钮被按下过的次数
                CopyOnWriteArrayList<Consumer<Integer>> actions = registeredButtonToggle.get(button);
                if (actions != null) { // 确保列表存在
                    for (Consumer<Integer> action : actions) {
                        action.accept(buttonState.computeIfAbsent(button, (ignored) -> 0));
                    }
                }
            }
            if (button.wasJustReleased(this)) {
                for (Runnable action :
                        Objects.requireNonNull(registeredButtonRelease.get(button))) {
                    action.run();
                }
            }
        }
    }

    public ListEventGamepad onPress(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonPress.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onPress(Buttons button, Consumer<Integer> action) {
        Objects.requireNonNull(registeredButtonToggle.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonPress.get(button)).
                add(new AsyncRunnable(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Consumer<Integer> action) {
        Objects.requireNonNull(registeredButtonToggle.get(button)).
                add(new AsyncConsumer<>(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Runnable action, Executor executor) {
        Objects.requireNonNull(
                registeredButtonPress.get(button)).add(new AsyncRunnable(action, executor)
        );
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button,
                                         Consumer<Integer> action,
                                         Executor executor) {
        Objects.requireNonNull(
                registeredButtonToggle.get(button)).add(new AsyncConsumer<>(action, executor)
        );
        return this;
    }

    public ListEventGamepad onRelease(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonRelease.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onReleaseAsync(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonRelease.get(button)).
                add(new AsyncRunnable(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onReleaseAsync(Buttons button, Runnable action, Executor executor) {
        Objects.requireNonNull(
                registeredButtonRelease.get(button)).add(new AsyncRunnable(action, executor)
        );
        return this;
    }
}