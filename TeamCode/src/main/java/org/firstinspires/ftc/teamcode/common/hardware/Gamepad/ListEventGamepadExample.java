package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

public class ListEventGamepadExample extends LinearOpMode {
    public final ListEventGamepad gamepad = new ListEventGamepad();

    @Override
    public void runOpMode() {
        Buttons.A
                .onPress(gamepad, () -> telemetry.addLine("Button.A - Pressed"))
                .onRelease(gamepad, () -> telemetry.addLine("Button.A - Released"));
    }
}
