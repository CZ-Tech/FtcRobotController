package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
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

    private final Map<Buttons, CopyOnWriteArrayList<Consumer<Boolean>>>
            registeredButtonChange = initEventMap(new HashMap<>());

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
                for (Consumer<Boolean> action :
                        Objects.requireNonNull(registeredButtonChange.get(button))) {
                    action.accept(PRESS);
                }
            }
            if (button.wasJustReleased(this)) {
                for (Runnable action :
                        Objects.requireNonNull(registeredButtonRelease.get(button))) {
                    action.run();
                }
                for (Consumer<Boolean> action :
                        Objects.requireNonNull(registeredButtonChange.get(button))) {
                    action.accept(RELEASE);
                }
            }
        }
    }

    public ListEventGamepad onButtonPress(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonPress.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onButtonPressAsync(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonPress.get(button)).
                add(new AsyncRunnable(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onButtonPressAsync(Buttons button, Runnable action, Executor executor) {
        Objects.requireNonNull(
                registeredButtonPress.get(button)).add(new AsyncRunnable(action, executor)
        );
        return this;
    }

    public ListEventGamepad onButtonRelease(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonRelease.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onButtonReleaseAsync(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonRelease.get(button)).
                add(new AsyncRunnable(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onButtonReleaseAsync(Buttons button, Runnable action, Executor executor) {
        Objects.requireNonNull(
                registeredButtonRelease.get(button)).add(new AsyncRunnable(action, executor)
        );
        return this;
    }

    public ListEventGamepad onButtonChange(Buttons button, Consumer<Boolean> action) {
        Objects.requireNonNull(registeredButtonChange.get(button)).add(action);
        return this;
    }

    public ListEventGamepad onButtonChangeAsync(Buttons button, Consumer<Boolean> action) {
        Objects.requireNonNull(registeredButtonChange.get(button)).
                add(new AsyncConsumer<>(action, getexecutor()));
        return this;
    }

    public ListEventGamepad onButtonChangeAsync(Buttons button, Consumer<Boolean> action, Executor executor) {
        Objects.requireNonNull(
                registeredButtonChange.get(button)).add(new AsyncConsumer<>(action, executor)
        );
        return this;
    }
}