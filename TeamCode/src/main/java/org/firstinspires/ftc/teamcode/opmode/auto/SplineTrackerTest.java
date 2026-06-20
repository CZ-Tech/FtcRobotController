package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.common.command.SplineTracker;

/**
 * A simple auto OpMode that drives a rectangular path using {@link SplineTracker}
 * to exercise spline following, heading control, and waypoint callbacks.
 *
 * <p>Path: forward → strafe right → turn 90° → forward → stop.</p>
 */

@Autonomous(name = "SplineTrackerTest", group = "Test")
public class SplineTrackerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(this);

        SplineTracker tracker = new SplineTracker(robot);

        telemetry.addData("Status", "Ready. DEBUG=" + Globals.DEBUG);
        telemetry.update();

        robot.odo.resetPosAndIMU();

        waitForStart();
        if (!opModeIsActive()) return;

        // ── Drive a simple rectangular path ──────────────────────────
        tracker
            .startMove(0, 0)                                         // 起点
            .addPoint(24,  0,  1, 0,  0,  () -> log("A: forward done"))
            .addPoint(24, 24,  0, 1,  0,  () -> log("B: strafe done"))
            .addPoint(24, 24,  0, 0, 90,  () -> log("C: turn to 90°"))
            .addPoint(0,  24, -1, 0, 90,  () -> log("D: back done"))
            .addPoint(0,   0,  0,-1,  0,  () -> log("E: return to start"))
            .stopMotor();

        log("Path complete. Stopped.");
        robot.waitFor(2000);

        TaskLoopFrame.stopAndClearAll();
    }

    private void log(String msg) {
        telemetry.addData("Waypoint", msg);
        telemetry.update();
    }
}