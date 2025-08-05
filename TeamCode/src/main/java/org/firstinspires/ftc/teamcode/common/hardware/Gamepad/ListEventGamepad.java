package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

public class ListEventGamepad extends Gamepad {

    private Executor executor = null;

    private final ScheduledExecutorService pollingExecutor =
            Executors.newSingleThreadScheduledExecutor();

    // --- Event Listener Maps ---
    private final Map<Buttons, CopyOnWriteArrayList<Runnable>>
            registeredButtonPress = initEventMap();
    private final Map<Buttons, CopyOnWriteArrayList<Runnable>>
            registeredButtonRelease = initEventMap();
    private final Map<Buttons, CopyOnWriteArrayList<Consumer<Integer>>>
            registeredButtonToggle = initEventMap();
    private final Map<Buttons, CopyOnWriteArrayList<Consumer<Integer>>>
            registeredWhenButtonDown = initEventMap();

    // --- State Tracking ---
    private final ConcurrentHashMap<Buttons, Integer> buttonState = new ConcurrentHashMap<>();
    private final Set<Buttons> watchedButtons = EnumSet.noneOf(Buttons.class);

    private <T> Map<Buttons, CopyOnWriteArrayList<T>> initEventMap() {
        Map<Buttons, CopyOnWriteArrayList<T>> map = new HashMap<>();
        for (Buttons button : Buttons.values()) {
            map.put(button, new CopyOnWriteArrayList<>());
        }
        return map;
    }

    private synchronized Executor getExecutor() {
        if (executor == null) {
            // Use a fixed thread pool for better resource management
            executor = Executors.newFixedThreadPool(4);
        }
        return executor;
    }

    public ListEventGamepad() {
        // Poll at a slightly higher rate for better responsiveness
        pollingExecutor.scheduleAtFixedRate(this::pollControllerState, 0, 20, TimeUnit.MILLISECONDS);
    }

    public void pollControllerState() {
        // Iterate only over buttons that have listeners attached
        for (Buttons button : watchedButtons) {
            // Handle "just pressed" events
            if (button.wasJustPressed(this)) {
                // Regular press listeners
                for (Runnable action : Objects.requireNonNull(registeredButtonPress.get(button))) {
                    action.run();
                }

                // Toggle listeners (tracks press count)
                CopyOnWriteArrayList<Consumer<Integer>> toggleActions = registeredButtonToggle.get(button);
                if (toggleActions != null && !toggleActions.isEmpty()) {
                    Integer pressCount = buttonState.compute(button, (k, v) -> (v == null) ? 1 : v + 1);
                    for (Consumer<Integer> action : toggleActions) {
                        action.accept(pressCount);
                    }
                }
            }

            // Handle "just released" events
            if (button.wasJustReleased(this)) {
                for (Runnable action : Objects.requireNonNull(registeredButtonRelease.get(button))) {
                    action.run();
                }
            }

            // Handle "while down" events
            if (button.ifPressed(this)) {
                CopyOnWriteArrayList<Consumer<Integer>> whenDownActions = registeredWhenButtonDown.get(button);
                if (whenDownActions != null && !whenDownActions.isEmpty()) {
                    // Get the current press count, defaulting to 1 if not already pressed
                    Integer pressCount = buttonState.getOrDefault(button, 1);
                    for (Consumer<Integer> action : whenDownActions) {
                        action.accept(pressCount);
                    }
                }
            }
        }
    }
    
    private void watch(Buttons button) {
        watchedButtons.add(button);
    }

    public ListEventGamepad onPress(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonPress.get(button)).add(action);
        watch(button);
        return this;
    }

    public ListEventGamepad onPress(Buttons button, Consumer<Integer> action) {
        Objects.requireNonNull(registeredButtonToggle.get(button)).add(action);
        watch(button);
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Runnable action) {
        onPress(button, new AsyncRunnable(action, getExecutor()));
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Consumer<Integer> action) {
        onPress(button, new AsyncConsumer<>(action, getExecutor()));
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Runnable action, Executor executor) {
        onPress(button, new AsyncRunnable(action, executor));
        return this;
    }

    public ListEventGamepad onPressAsync(Buttons button, Consumer<Integer> action, Executor executor) {
        onPress(button, new AsyncConsumer<>(action, executor));
        return this;
    }

    public ListEventGamepad onRelease(Buttons button, Runnable action) {
        Objects.requireNonNull(registeredButtonRelease.get(button)).add(action);
        watch(button);
        return this;
    }

    public ListEventGamepad onReleaseAsync(Buttons button, Runnable action) {
        onRelease(button, new AsyncRunnable(action, getExecutor()));
        return this;
    }

    public ListEventGamepad onReleaseAsync(Buttons button, Runnable action, Executor executor) {
        onRelease(button, new AsyncRunnable(action, executor));
        return this;
    }

    public ListEventGamepad whenDown(Buttons button, Consumer<Integer> action) {
        Objects.requireNonNull(registeredWhenButtonDown.get(button)).add(action);
        watch(button);
        return this;
    }

    public ListEventGamepad whenDownAsync(Buttons button, Consumer<Integer> action) {
        whenDown(button, new AsyncConsumer<>(action, getExecutor()));
        return this;
    }
    
    // --- Helper classes for async operations ---
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
