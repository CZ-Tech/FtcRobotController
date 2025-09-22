package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Globals {
    public static boolean DEBUG=false;
    public static boolean DEBUG_RUN_FUNC = false;
    public static boolean DO_NOT_RUN_FUNC = false;


    // 反注释下方代码进入调试模式，用于测试每个电机方向。
    // Globals.DEBUG = true;
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

    public static String LeftFrontMotor="lfm";
    public static String RightFrontMotor="rfm";
    public static String LeftBackMotor="lbm";
    public static String RightBackMotor="rbm";
    public static DcMotor.Direction LeftFrontMotorDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction RightFrontMotoDirection = DcMotor.Direction.REVERSE;
    public static DcMotor.Direction LeftBackMotorDirection = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RightBackMotorDirection = DcMotor.Direction.FORWARD;

    //  设置里程计吊舱相对于跟踪点的位置偏移
    //  @param xOffset X吊舱偏移量（毫米），中心左侧为正，右侧为负
    //  @param yOffset Y吊舱偏移量（毫米），中心前方为正，后方为负
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

    public static double odoXOffset=0;
    public static double odoYOffset=-95.99872249;
}