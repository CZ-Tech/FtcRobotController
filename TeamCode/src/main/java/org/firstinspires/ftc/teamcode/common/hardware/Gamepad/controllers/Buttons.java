package org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * 这里储存了手柄上每一个键与Gamepad类的名称
 * 同时与sdk中Gamepad类提供的状态变量一一对应
 * 还为每个按钮存在的不同行为提供了不同参数获取方法
 * 此处的注释基本是Gamepad中的注释的直译，如有错误敬请修改
 *
 * ps: 张老师你说得对，按钮一点也不多，也就三十个对吧
 */
public enum Buttons {
    // D-pad buttons
    DPAD_UP {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.dpad_up; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.dpadUpWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.dpadUpWasReleased(); }
    },
    DPAD_DOWN {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.dpad_down; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.dpadDownWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.dpadDownWasReleased(); }
    },
    DPAD_LEFT {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.dpad_left; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.dpadLeftWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.dpadLeftWasReleased(); }
    },
    DPAD_RIGHT {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.dpad_right; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.dpadRightWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.dpadRightWasReleased(); }
    },
    A {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.a; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.aWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.aWasReleased(); }
    },
    B {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.b; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.bWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.bWasReleased(); }
    },
    X {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.x; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.xWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.xWasReleased(); }
    },
    Y {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.y; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.yWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.yWasReleased(); }
    },
    // Center/Special buttons
    GUIDE {// 通常是控制器中间的大按钮。操作系统可能会在这个按钮被发送到应用程序之前捕获它；
        // 这样的话，你就永远无法收到它被按下的信号。
        public boolean ifPressed(Gamepad gamepad) { return gamepad.guide; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.guideWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.guideWasReleased(); }
    },
    START {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.start; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.startWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.startWasReleased(); }
    },
    BACK {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.back; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.backWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.backWasReleased(); }
    },
    // 肩键 (Bumper buttons)（保险杠按钮;)）
    LEFT_BUMPER {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.left_bumper; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.leftBumperWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.leftBumperWasReleased(); }
    },
    RIGHT_BUMPER {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.right_bumper; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.rightBumperWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.rightBumperWasReleased(); }
    },
    LEFT_STICK_BUTTON {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.left_stick_button; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.leftStickButtonWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.leftStickButtonWasReleased(); }
    },
    RIGHT_STICK_BUTTON {
        public boolean ifPressed(Gamepad gamepad) { return gamepad.right_stick_button; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.rightStickButtonWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.rightStickButtonWasReleased(); }
    },
    // PS4支持 - face按钮 (这都啥玩意啊)
    CIRCLE { // Mapped to B in SDK Gamepad aliases
        public boolean ifPressed(Gamepad gamepad) { return gamepad.circle; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.circleWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.circleWasReleased(); }
    },
    CROSS { // Mapped to A
        public boolean ifPressed(Gamepad gamepad) { return gamepad.cross; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.crossWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.crossWasReleased(); }
    },
    TRIANGLE { // Mapped to Y
        public boolean ifPressed(Gamepad gamepad) { return gamepad.triangle; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.triangleWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.triangleWasReleased(); }
    },
    SQUARE { // Mapped to X
        public boolean ifPressed(Gamepad gamepad) { return gamepad.square; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.squareWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.squareWasReleased(); }
    },
    SHARE { // Mapped to BACK
        public boolean ifPressed(Gamepad gamepad) { return gamepad.share; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.shareWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.shareWasReleased(); }
    },
    OPTIONS { // Mapped to START
        public boolean ifPressed(Gamepad gamepad) { return gamepad.options; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.optionsWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.optionsWasReleased(); }
    },
    PS { // Mapped to GUIDE
        public boolean ifPressed(Gamepad gamepad) { return gamepad.ps; }
        public boolean wasJustPressed(Gamepad gamepad) { return gamepad.psWasPressed(); }
        public boolean wasJustReleased(Gamepad gamepad) { return gamepad.psWasReleased(); }
    };

    public abstract boolean ifPressed(Gamepad gamepad);
    public abstract boolean wasJustPressed(Gamepad gamepad);
    public abstract boolean wasJustReleased(Gamepad gamepad);
}
