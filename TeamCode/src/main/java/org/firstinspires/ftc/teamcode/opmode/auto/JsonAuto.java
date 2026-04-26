package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.dirve.PinpointTrajectory;
import org.firstinspires.ftc.teamcode.common.dirve.TrajectoryLoader;

@Autonomous(name = "JsonAuto")
public class JsonAuto extends LinearOpMode {
    public static String autoStr = "[\n" +
            "    {\n" +
            "        \"x\": -1.010652780532837,\n" +
            "        \"dx\": 1.6878557205200195,\n" +
            "        \"y\": 25.60612392425537,\n" +
            "        \"dy\": -53.40950012207031,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 0.0\n" +
            "    },\n" +
            "    {\n" +
            "        \"x\": 0.98760986328125,\n" +
            "        \"dx\": -45.575218200683594,\n" +
            "        \"y\": 0.12743377685546875,\n" +
            "        \"dy\": -37.115044593811035,\n" +
            "        \"heading\": 0.0,\n" +
            "        \"dHeading\": 0.0,\n" +
            "        \"duration\": 1.0,\n" +
            "        \"time\": 1.0\n" +
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

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//