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

    public int getBaseAprilTag(){
        if (color == BLUE.getColor()) return 20;
        return 24;
    }

    public double getBaseAngle(){
        if (color == BLUE.getColor()) return 54;
        return -54;
    }

    public Pose2D getGoalPos(){
        if (this == RED) return Globals.RED_GOAL_POS;
        else if (this == BLUE) return Globals.BLUE_GOAL_POS;
        else return Globals.BLUE_GOAL_POS;
    }
}