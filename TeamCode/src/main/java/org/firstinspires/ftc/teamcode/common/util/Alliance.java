package org.firstinspires.ftc.teamcode.common.util;

public enum Alliance {
    NONE(0),
    RED(1),
    BLUE(2);
    private final int color;

    Alliance(int color) {
        this.color = color;
    }

    public int getColor() {
        return color;
    }
}