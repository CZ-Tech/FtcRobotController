package org.firstinspires.ftc.teamcode.common.hardware.Gamepad.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

public enum LinearReturns {
    // Trigger axes 线性扳机
    LEFT_TRIGGER {
        public float getFloatValue(Gamepad gamepad) {
            return gamepad.left_trigger;
        }
    },
    RIGHT_TRIGGER {
        public float getFloatValue(Gamepad gamepad) {
            return gamepad.right_trigger;
        }
    };

    public abstract float getFloatValue(Gamepad gamepad);
}
