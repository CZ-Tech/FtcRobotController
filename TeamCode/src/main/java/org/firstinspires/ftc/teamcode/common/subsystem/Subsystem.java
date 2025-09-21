package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.Robot;

public class Subsystem {
    private final Robot robot;

    public final Intaker intaker;
    public final Thrower thrower;
    public Subsystem(Robot robot) {
        this.robot = robot;
        this.intaker = new Intaker(robot);
        this.thrower = new Thrower(robot);
    }
}
