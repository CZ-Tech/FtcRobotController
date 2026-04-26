package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;

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

    public Pose2D getGoalPos(){
        if (color == BLUE.getColor()) return Globals.BLUE_GOAL_POS;
        return Globals.RED_GOAL_POS;
    }
}