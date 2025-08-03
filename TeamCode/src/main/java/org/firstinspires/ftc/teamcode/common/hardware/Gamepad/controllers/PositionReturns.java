package org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

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
}
