package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.util.Alliance;
import org.firstinspires.ftc.teamcode.common.Robot;

public class Duo extends LinearOpMode {
    Robot robot = Robot.INSTANCE;
    double speedMul = 1;
    Alliance teamColor = Alliance.NONE;
    Boolean AsianDriverMode = true;

    @Override
    public void runOpMode() {

        robot.init(this);
        waitForStart();
        robot.odometry.resetPos();

        double drive = 0;
        double strafe = 0;
        double turn = 0;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (AsianDriverMode) {
                // 日本手
                drive = -sss(gamepad1.left_stick_y);  //Note: pushing stick forward gives negative value
                strafe = sss(gamepad1.left_stick_x);//日本手
                turn = sss(gamepad1.right_stick_x * (1 + 0.5 * (gamepad1.left_stick_y * gamepad1.left_stick_y) + 0.5 * (gamepad1.left_stick_x * gamepad1.left_stick_x)));
            }
            robot.drivetrain.driveRobotFieldCentric(drive, strafe, turn);
            robot.odometry.update();
            if (gamepad1.share && gamepad1.options) {
                robot.drivetrain.resetYaw();
            }
            telemetry.addData("G1 lx ly rx ry", "%.2f %.2f %.2f %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData(">", "Robot Heading = %4.0f", robot.drivetrain.getHeading());
            telemetry.addData(">", "XPos = %4.0f  YPos = %4.0f", robot.odometry.X_Pos, robot.odometry.Y_Pos);
            telemetry.addData(">", "△X = %4.0f  △Y = %4.0f", robot.odometry.delta_x, robot.odometry.delta_y);
            telemetry.update();
        }

    }

    private double sss(double v) {
        v = v * speedMul;
        if (v > 0.0) { //若手柄存在中位漂移或抖动就改0.01
            v = 0.87 * v * v * v + 0.13;//0.13是23-24赛季底盘启动需要的功率
        } else if (v < 0.0) { //若手柄存在中位漂移或抖动就改-0.01
            v = 0.87 * v * v * v - 0.13; //三次方是摇杆曲线
        } else {
            // XBOX和罗技手柄死区较大无需设置中位附近
            // 若手柄存在中位漂移或抖动就改成 v*=13
            // 这里的13是上面的0.13/0.01=13
            v = 0;
        }
        return v;
    }

    @TeleOp(name = "Duo🔴", group = "Duo")
    public static class DuoR extends Duo {
        @Override
        public void runOpMode() {
            teamColor = Alliance.RED;
            super.runOpMode();
        }
    }

    @TeleOp(name = "Duo🔵", group = "Duo")
    public static class DuoB extends Duo {
        @Override
        public void runOpMode() {
            teamColor = Alliance.BLUE;
            super.runOpMode();
        }
    }
}

