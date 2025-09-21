package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Duo extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(this);
        telemetry.addLine("Robot ready!");
        waitForStart();
        while (opModeIsActive()) {
            // update 必须每次循环最开始执行，并且整个循环内每个手柄只能执行一次。
            robot.gamepad1.update();

            robot.gamepad1
                    .keyPress("a", () -> telemetry.addLine("Button.A - Pressed"))
                    .keyPress("left_stick_button", () -> telemetry.addLine("Button.LEFT_STICK_BUTTON - Pressed"))
                    .keyPress("dpad_up", () -> telemetry.addLine("Button.DPAD_UP - Pressed"))
                    .keyPress("left_bumper", () -> telemetry.addLine("Button.LEFT_BUMPER - Pressed"))
                    .keyUp("a", () -> telemetry.addLine("Button.A - KeyUp"))
                    .keyDown("a", () -> telemetry.addLine("Button.A - KeyDown"))
                    .keyToggle("cross",
                            () -> telemetry.addLine("Button.A - KeyToggle 1"),
                            () -> telemetry.addLine("Button.A - KeyToggle 2")
                    )
            ;

            robot.odoDrivetrain.driveRobotFieldCentric(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            telemetry.update();
        }
    }

    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoRed extends Duo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoBlue extends Duo {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }
}

