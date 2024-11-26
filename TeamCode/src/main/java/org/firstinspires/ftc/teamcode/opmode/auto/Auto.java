package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Alliance;


public class Auto extends LinearOpMode {
    Robot robot = Robot.INSTANCE;

    @Override
    public void runOpMode() {
        //Initialization
        robot.init(this);

        //Wait for the 'Start' button pressed
        waitForStart();

        //Main methods
        robot.drivetrain.resetYaw();
        robot.drivetrain
                .driveStraight(24.0, 0.0, 1.0)
                .sleep(1000)
                .driveStrafe(24.0, 0.0, 1.0)
                .sleep(1000)
                .turnToHeading(90, 0.5)
        ;

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
