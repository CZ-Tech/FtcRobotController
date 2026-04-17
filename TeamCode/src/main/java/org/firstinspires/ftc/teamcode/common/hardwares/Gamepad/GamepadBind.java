package org.firstinspires.ftc.teamcode.common.hardwares.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers.Buttons;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME) // 必须是 RUNTIME 才能在反射中被读取
@Target(ElementType.METHOD)         // 只能作用于方法
public @interface GamepadBind {
    int gamepad() default 1;               // 指定手柄 (1 或 2)
    Buttons button();                      // 指定按键
    GamepadEx.ActionType actionType(); // 默认触发方式为按下
    String id() default "";                // 可选：指定ID（留空则为匿名绑定）

    // 允许指定该绑定生效的 OpMode 类。留空则代表所有 OpMode 均生效。
    Class<?>[] activeIn() default {};
}
