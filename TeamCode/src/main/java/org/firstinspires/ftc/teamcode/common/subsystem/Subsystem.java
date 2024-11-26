package org.firstinspires.ftc.teamcode.common.subsystem;

public enum Subsystem {
    INSTANCE;

    public final Intaker intaker;
    public final Thrower thrower;

    Subsystem() {
        this.intaker = Intaker.INSTANCE;
        this.thrower = Thrower.INSTANCE;
    }
}
