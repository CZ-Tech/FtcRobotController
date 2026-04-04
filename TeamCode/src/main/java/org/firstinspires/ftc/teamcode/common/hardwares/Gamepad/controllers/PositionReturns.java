package org.firstinspires.ftc.teamcode.common.hardwares.Gamepad.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Globals;

public enum PositionReturns {
    // 操纵杆
    LEFT_STICK {
        public float[] getPos(Gamepad gamepad) {
            return new float[] {gamepad.left_stick_x, gamepad.left_stick_y};
        }
    },
    RIGHT_STICK {
        public float[] getPos(Gamepad gamepad) {
            return new float[] {gamepad.right_stick_x, gamepad.right_stick_y};
        }
    },

    // PS4支持 - 触控板（手柄上还有这等东西？？？
    TOUCHPAD_FINGER1 {
        public float[] getPos(Gamepad gamepad) {
            return new float[] {gamepad.touchpad_finger_1_x, gamepad.touchpad_finger_1_y};
        }
    },

    TOUCHPAD_FINGER2 {
        public float[] getPos(Gamepad gamepad) {
            return new float[] {gamepad.touchpad_finger_2_x, gamepad.touchpad_finger_2_y};
        }
    };

    public abstract float[] getPos(Gamepad gamepad);

    /**
     * 将摇杆输入转换为平滑响应的输出
     * 基于定义在[0,1]的函数 f(x) = a*x^n + (1-a)*x
     * 通过原点对称扩展得到 [-1,1] 上的完整奇函数
     * a 控制非线性部分的权重 （该参数几乎只在0.9附近可用）
     * n 为非线性程度参数（即高次项次数），n > 0，越大在接近边界时增速越快
     *
     * @param x 原始摇杆输入，范围 [-1, 1]
     * @return 调整后的摇杆输出
     */
    public static double applyResponseCurve(double x) {
        x = Math.max(-1.0, Math.min(1.0, x));

        // 使用 signum 函数处理奇函数特性
        double absX = Math.abs(x);

        double a = Globals.StickResponseCurve_a;
        double n = Globals.StickResponseCurve_n;

        // 特殊情况的快速路径
        if (absX == 0.0) return 0.0;
        if (absX == 1.0) return x;

        // 正半轴计算公式
        double yPos = a * Math.pow(absX, n) + (1.0 - a) * absX;

        return Math.copySign(yPos, x);
    }
}
