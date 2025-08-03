package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

public class ListEventGamepadExample extends LinearOpMode {
    public final ListEventGamepad gamepad = new ListEventGamepad();

    @Override
    public void runOpMode() {
        gamepad.onPress(Buttons.A, () -> telemetry.addLine("Button.A - Pressed"));
        gamepad.onPress(Buttons.LEFT_STICK_BUTTON,
                () -> telemetry.addLine("Button.LEFT_STICK_BUTTON - Pressed"));
        gamepad.onPress(Buttons.DPAD_UP, () -> telemetry.addLine("DPAD_UP pressed"))
                .onPressAsync(Buttons.B, () -> {  // 检测第二个键是否被按下
                    while (true) {
                        if (Buttons.Y.wasJustPressed(gamepad))
                            telemetry.addLine("B and Y was pressed");
                    }
                });
    }
}
