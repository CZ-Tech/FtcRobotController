package org.firstinspires.ftc.teamcode.common.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers.Buttons;

public class ListEventGamepadExample extends LinearOpMode {
    public final ListEventGamepad gamepad = Robot.getInstance().gamepad1;

    @Override
    public void runOpMode() {
        gamepad.listen(Buttons.A)
                .onPress(() -> telemetry.addLine("Button.A - Pressed"))
                .onRelease(() -> telemetry.addLine("Button.A - Released"));
    }
}
