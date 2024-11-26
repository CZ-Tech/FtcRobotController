package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class GamepadEx extends Gamepad {
    private final Gamepad gamepad;
    private final Gamepad currGamepad = new Gamepad();
    private final Gamepad prevGamepad = new Gamepad();
    private final HashMap<String, Boolean> keyState = new HashMap<>();

    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public GamepadEx update() {
        prevGamepad.copy(currGamepad);
        currGamepad.copy(gamepad);
        return this;
    }

    public GamepadEx keyPress(String key, Runnable fn) {
        if (getKey(currGamepad, key)) fn.run();
        return this;
    }

    public GamepadEx keyDown(String key, Runnable fn) {
        if (getKey(currGamepad, key) && !getKey(prevGamepad, key)) fn.run();
        return this;
    }

    public GamepadEx keyUp(String key, Runnable fn) {
        if (!getKey(currGamepad, key) && getKey(prevGamepad, key)) fn.run();
        return this;
    }

    public GamepadEx keyToggle(String key, Runnable fn1, Runnable fn2) {
        keyState.putIfAbsent(key, false);
        if (getKey(currGamepad, key) && !getKey(prevGamepad, key)){
            keyState.computeIfPresent(key, (k, v) -> !v);
            if (keyState.getOrDefault(key, false)){
                fn1.run();
            }else{
                fn2.run();
            }
        }
        return this;
    }

    private boolean getKey(Object obj, String keyName) {
        try {
            return (boolean) obj.getClass().getDeclaredField(keyName).get(obj);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
