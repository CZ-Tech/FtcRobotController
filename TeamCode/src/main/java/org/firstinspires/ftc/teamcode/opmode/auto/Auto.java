package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.PinpointTrajectory;

public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 1. 初始化你的机器人硬件类 (你的 Robot 类需要包含 odo 和 odoDrivetrain)
        Robot robot = new Robot();
        robot.init(this);

        // 2. 实例化轨迹控制器
        PinpointTrajectory trajectory = new PinpointTrajectory(robot);

        waitForStart();

        if (opModeIsActive()) {

            // 3. 开始链式调用编写轨迹
            trajectory
                    .setMode(PinpointTrajectory.Mode.SEPARATED) // 设置为分段相对时间模式
                    .reset()

                    // 起点：X=0, Y=0, X方向初速度=0, Y方向初速度=0
                    .startMove(0, 0, 0, 0)

                    // 路径点 1：花费 2.0 秒，移动到 (24, 0)
                    // dx=10, dy=0 表示经过该点时保持 X 方向的运动趋势 (不减速)
                    .addPoint(2.0, 10, 0, 10, 0)

                    // 路径点 2：花费 2.0 秒，平滑移动到 (48, 24)
                    // 同时把机器人朝向 (Heading) 转到 90度
                    .addPoint(2.0, 24, 12, 0, 10, 90)

                    // 路径点 3：花费 1.5 秒，移动到 (48, 48) 并完全停下 (dx=0, dy=0)
                    // 并且在到达该点后，执行一个回调操作（比如放下预装的样本/标尺）
                    .addPoint(1.5, 48, 48, 0, 0, 90, () -> {
                        // 这里写你要执行的机械臂或抓手代码
                        // 例如：robot.claw.open();
                    })

                    // 路径点 4：原地等待 (利用 addTime 延时而不改变目标点)
                    .addTime(1.0)

                    // 结束动作：刹车并停止底盘电机
                    .stopMotor();
        }
    }
}
