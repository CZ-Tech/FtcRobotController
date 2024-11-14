package org.firstinspires.ftc.teamcode.common.subsystem;


import org.firstinspires.ftc.teamcode.common.Robot;

public enum Subsystem {
    INSTANCE;
    private final Robot robot = Robot.INSTANCE;

    public Intaker intaker;
    public Thrower thrower;

    Subsystem() {
        this.intaker = Intaker.INSTANCE;
        this.thrower = Thrower.INSTANCE;
    }
}
