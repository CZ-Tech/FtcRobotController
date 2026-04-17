package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class Globals {
    // TODO 将此处改为true进入调试模式，用于测试每个电机方向。
    public static boolean DEBUG=true;
    // 一号手柄按住share键或者back键测试
    /**
     * Xbox/PS4 Button - Motor
     *   X / ▢         - Left  Front
     *   Y / Δ         - Right Front
     *   B / O         - Right Back
     *   A / X         - Left  Back
     *                                    The buttons are mapped to match the wheels spatially if you
     *                                    were to rotate the gamepad 45deg°. x/square is the front left
     *                    ________        and each button corresponds to the wheel as you go clockwise
     *                   / ______ \
     *     ------------.-'   _  '-..+              Front of Bot
     *              /   _  ( Y )  _  \                  ^
     *             |  ( X )  _  ( B ) |      Left Front  \    Right Front
     *        ___  '.      ( A )     /|       Wheel       \      Wheel
     *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
     *     |       |                 |                      \
     *      '.___.' '.               |          Left Back    \   Right Back
     *               '.             /             Wheel       \    Wheel
     *                \.          .'              (A/X)        \   (B/O)
     *                  \________/
     *  https://rr.brott.dev/docs/v1-0/tuning/
     */

    //底盘电机名称
    public static String LeftFrontMotor = "lfm";
    public static String RightFrontMotor = "rfm";
    public static String LeftBackMotor = "lbm";
    public static String RightBackMotor = "rbm";

    //摄像头
    public static boolean UseVisionLocate = false;  // 如果你没装摄像头，请将此处设为false
    public static String limelight = "Webcam 1";
    public static String logiC270 = "Webcam 2";

    // 设置电机正方向，确保默认向前
    public static DcMotor.Direction LeftFrontMotorDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RightFrontMotoDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction LeftBackMotorDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RightBackMotorDirection = DcMotor.Direction.REVERSE;

    //  设置里程计吊舱相对于跟踪点的位置偏移
    //  @param xOffset 检测前后运动的吊舱偏移量（毫米），中心左侧为正，右侧为负
    //  @param yOffset 检测左右运动的吊舱偏移量（毫米），中心前方为正，后方为负
    //  +++++++++++++++++++++++++++++++
    //  +                        |||  +
    //  +               xOffset  |||  +
    //  +             <--------->     +
    //  +             ● Center        +        FIXME:Center指的是机器的旋转中心
    //  +             |               +
    //  +             | yOffset       +
    //  +             V               +
    //  +           =====             +
    //  +           =====             +
    //  +++++++++++++++++++++++++++++++
    public static String odoName = "odo";
    public static double odoXOffset = 0;
    public static double odoYOffset = -0.99872249;  // -95.99872249
    // TODO:确认自己使用的里程计类型
    public static GoBildaPinpointDriver.GoBildaOdometryPods odoType = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // 确认odo编码器方向，向前读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoXDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // 确认odo编码器方向，向左读数增加。
    public static GoBildaPinpointDriver.EncoderDirection odoYDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;


    // 软PID循环速率 (如果你使用util包中的的pid的话)
    public static int PID_THREAD_Hz = 120;

    //
    public static Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, 72, AngleUnit.DEGREES, 0);
    public static Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.INCH, -72, -72, AngleUnit.DEGREES, 0);

    public static double HEAD_SHOOT_ANGLE_DEG = 180;  // 定义的车头和发射之间的夹角

    // 设置手柄摇杆位置和输出功率间的关系
    // 基于定义在[0,1]的函数 f(x) = a*x^n + (1-a)*x
    // 通过原点对称扩展得到 [-1,1] 上的完整奇函数
    // a 控制非线性部分的权重 （该参数几乎只在0.9附近可用）(千万别乱改！先去desmos里看看！！）
    // n 为非线性程度参数（即高次项次数），n > 0，越大在接近边界时增速越快
    public static double StickResponseCurve_a = 0.9;
    public static double StickResponseCurve_n = 2;
}




