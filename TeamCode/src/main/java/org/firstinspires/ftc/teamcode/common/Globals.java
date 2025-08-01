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
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.没有外部齿轮转动
    public static final double WHEEL_DIAMETER_INCHES = 4.00;     // 96cm For figuring circumference
    public static double HEADING_THRESHOLD = 1.0;    // 在移动到下一步时头部必须离目标有多近

    // 定义移动常量. 定义public类型以在opmode中调用
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double P_DRIVE_GAIN = 0.03;     // 更大的数值使它更加灵敏, 但会让它更不稳定
    public static final double P_STRAFE_GAIN = 0.025;   // 平移速度控制 "Gain".
    public static final double P_TURN_GAIN = 0.02;     // 更大的数值使它更加灵敏, 但会让它更不稳定
    public static final double LATERAL_DISTANCE = 10.63;  //TODO:调试此值，让机器走准
    public static final double FORWARD_OFFSET = 5.12;  //TODO:调试此值，让机器走准
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


    public static final RevHubOrientationOnRobot orientationOnRobot=new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );


    // 以下是关于abs的数据, MOTORS_POWER_CURVE可不填，效果应该差不多
    // TODO 待测！
    public static final double[] MOTORS_POWER_CURVE = null;  // 四轮电机输出牵引力曲线,单位牛顿
                                                             // 对应功率从0~1
                                                             // 每个数值间对应功率相差大小相等
    public static final double MAXIMUM_STATIC_FRICTION = 35;  // 最大静摩擦力，单位牛顿，用于计算刹车距离

}