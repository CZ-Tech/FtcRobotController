package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.drive.TrajectoryLoader;

@Autonomous(name = "JsonAuto")
public class JsonAuto extends LinearOpMode {
    public static String autoStr = "[\n" +
            "    {\n" +
            "        \"x\": 48.02486991882324,\n" +
            "        \"dx\": 0.0,\n" +
            "        \"y\": -48.364423751831055,\n" +
            "        \"dy\": 0.0,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -49.03008508682251,\n" +
            "        \"dx\": 1.4241065979003906,\n" +
            "        \"y\": -23.415929794311523,\n" +
            "        \"dy\": 84.05582046508789,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 1.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": -47.7876091003418,\n" +
            "        \"dx\": -0.9734487533569336,\n" +
            "        \"y\": 23.957521438598633,\n" +
            "        \"dy\": 48.265485763549805,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 2.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 46.513275146484375,\n" +
            "        \"dx\": 44.74336266517639,\n" +
            "        \"y\": 47.915042877197266,\n" +
            "        \"dy\": -20.38938045501709,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 3.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 48.29734420776367,\n" +
            "        \"dx\": 1.5752220153808594,\n" +
            "        \"y\": -1.0194690227508545,\n" +
            "        \"dy\": -40.619468688964844,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 4.0\n" +
            "    }\n" +
            "]";


    @Override
    public void runOpMode() {
        // 1. 初始化你的机器人硬件类 (你的 Robot 类需要包含 odo 和 odoDrivetrain)
        Robot robot = new Robot();
        robot.init(this);

        // 2. 实例化轨迹控制器
        PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        waitForStart();

        if (opModeIsActive()) {
            TrajectoryLoader.executeJsonTrajectory(autoStr, trajectory);
        }
    }
}

