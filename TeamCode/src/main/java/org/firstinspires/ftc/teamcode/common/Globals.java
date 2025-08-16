package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Globals {
    // Rev HD Hex Motor (No Gearbox) : 28 counts/revolution
    // eg: GoBILDA 312 RPM (19.2:1) Yellow Jacket 537.7=28*19.2
    // 40:1 Rev 28*40=1120
    // 20:1 Rev 28*20=560
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    // [电机编码器参数说明]
    // Rev HD Hex 电机（无内置齿轮箱）: 每转有28个编码器计数（tick）。
    //
    // 示例 - 如何计算带齿轮箱电机的编码器计数：
    //   - GoBILDA 312 RPM 电机（黄夹克，减速比 19.2:1）的每转编码器计数为 537.7，计算方式为 28 * 19.2。
    //   - 40:1 减速比的 Rev 电机: 28 * 40 = 1120
    //   - 20:1 减速比的 Rev 电机: 28 * 20 = 560
    //
    // 你需要为你的机器人驱动系统计算“COUNTS_PER_INCH”（每英寸对应的编码器计数值）。
    // 为此，请先访问你所用电机的供应商网站，以确定其准确的“COUNTS_PER_MOTOR_REV”（电机输出轴每转的编码器计数值）。
    //
    // 如果使用了电机之外的外部齿轮传动，请根据需要设置“DRIVE_GEAR_REDUCTION”（外部驱动减速比）。
    //   - 例如，使用一个12齿的主动齿轮驱动一个24齿的从动齿轮时，减速比应设为 2.0 (24/12)。
    //   - 这种配置是“降速增扭”（Gearing Down），它会降低最终转速但能提供更大的扭矩。
    //   - 若要“增速减扭”（Gearing Up），则减速比应小于1.0。请注意，这可能会改变车轮的旋转方向。
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.没有外部齿轮转动
    public static final double WHEEL_DIAMETER_INCHES = 4.00;   // 车轮直径（单位：英寸），用于计算车轮周长
    public static double HEADING_THRESHOLD = 1.0;    // 在移动到下一步时头部必须离目标有多近
    public static double MIN_START_POWER = 0.05;  // 对于每个电机使机器人启动的最小最小功率

    // 定义移动常量. 定义public类型以在opmode中调用
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // 移动P增益值  更大的数值使机器人更加灵敏, 但会更不稳定
    public static final double P_DRIVE_GAIN = 0.03;     // 前进/后退方向的增益值
    public static final double P_STRAFE_GAIN = 0.025;   // 平移速度增益值
    public static final double P_TURN_GAIN = 0.02;     // 旋转速度增益值
    public static final double LATERAL_DISTANCE = 10.63;  // 轴距  //TODO:调试此值，让机器走准
    public static final double FORWARD_OFFSET = 5.12;  // 机器人的旋转中心到传感器(TODO?未确认)
                                                       // 在前进/后退轴上的距离
                                                       // TODO:调试此值，让机器走准

//  +++++++++++++++++++++++++++++++
//  +  |||                   |||  +
//  +  ||| LATERAL_DISTANCE  |||  +
//  +     <----------------->     +
//  +             ● Center        +        FIXME:Center指的是机器的旋转中心
//  +             |               +
//  +             | FORWARD_OFFSET+
//  +             V               +
//  +           =====             +
//  +           =====             +
//  +++++++++++++++++++++++++++++++

    //定义里程计相对于中心的偏差
    //148.027 -44.020
    public static final double X_OFFSET = 0;
    public static final double Y_OFFSET = -95.99872249;


    public static final RevHubOrientationOnRobot orientationOnRobot=new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

}