package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.util.OpModeState;


public class Auto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        //Initialization
        robot.init(this);
        robot.opModeState = OpModeState.Auto;

        robot.telemetry.addData("Status", "Waiting for start");
        robot.telemetry.update();

        robot.pinpointTrajectory.reset();
        //Wait for the 'Start' button pressed
        waitForStart();

        //Main methods
        //路径
        robot.pinpointTrajectory
                .setMode(PinpointTrajectory.Mode.TIMELINE)
                .startMove()

                //推第一个样本
                .addFunc(() -> telemetry.addLine("test1"))
                .addPoint(1.4, -32.5, 0, 0, 0, 0, ()->telemetry.addLine("test2"))
                .addTime(1.5)
                .addVelocity(50, 25)
                .addPoint(2.45, -25.864, 6.228, 0, 0, 90)

                //推第二个样本
                .addPoint(2.65, -28.35, 13.2, 0, 0, 116.134, () -> telemetry.addLine("test3"))
                .addTime(2.9)
                .addPoint(3.5, -14.638, 13, 0, 0, 45)

                //推第三个样本
                .addVelocity(-25, -30)
                .addPoint(4.25, -27.887, 19.4, 0, 0, 125)
                .addTime(4.55)
                .addPoint(5.35, -17.187, 19.546, 0, 0, 45)

                .stopMotor()
        ;

        //wait for stop
        while (opModeIsActive());

    }

    //Red alliance
    @Autonomous(name = "Auto🔴", group = "Auto", preselectTeleOp = "Duo🔴")
    public static class AutoRed extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    //Blue alliance
    @Autonomous(name = "Auto🔵", group = "Auto", preselectTeleOp = "Duo🔵")
    public static class AutoBlue extends Auto {
        @Override
        public void runOpMode() {
            robot.teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }
}
