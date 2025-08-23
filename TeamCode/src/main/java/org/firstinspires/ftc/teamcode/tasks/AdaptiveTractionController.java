package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.drive.AntiLockTractionSystem;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;

import java.util.Arrays;

public class AdaptiveTractionController {
    private GoBildaPinpointDataAsync odo;
    private AntiLockTractionSystem ATS;

    public static int VelocityCurveDotsCount = 128;

    private double[] targetPos = new double[3];
    private double[] speedOnTargetPos = new double[3];

    private final double[] lastVelocityLinePos1 = new double[2];
    private final double[] lastVelocityLinePos2 = new double[2];

    private final double[] currentVelocityLinePos1 = new double[2];
    private final double[] currentVelocityLinePos2 = new double[2];

    private Modes mode = Modes.RUN_ANTILOCK_ONLY;

    public static enum Modes {
        RUN_TO_POSITION,
        RUN_ANTILOCK_ONLY
    }

    public enum state {
        MIXED,
        BREAKING,
        ACCELERATING,  // 直线加速
        TURNING,
        CONSTANT,  // 匀速
        STOPPED
    }

    private AdaptiveTractionController(GoBildaPinpointDataAsync odo) {
        this.odo = odo;
        AntiLockTractionSystem.ATSConfig config = AntiLockTractionSystem.ATSConfig.getBuilder()
                .setAntislipPowerChangeFactor(Globals.ANTISLIP_POWER_CHANGE_FACTOR)
                .setAtsLockThreshold(Globals.ATS_LOCK_THRESHOLD)
                .setAtsPowerChangeStep(Globals.ATS_POWER_CHANGE_STEP)
                .setAtsStartPower(Globals.ATS_START_POWER)
                .setLateralDistance(Globals.LATERAL_DISTANCE)
                .setWheelbase(Globals.WHEELBASE)
                .setCountsPerMotorRev(Globals.COUNTS_PER_MOTOR_REV)
                .setWheelDiameterInches(Globals.WHEEL_DIAMETER_INCHES)
                .build();
        ATS = new AntiLockTractionSystem(odo, config);
    }

    public void update() {

    }

    public void setTargetPos(double x, double y, double heading) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = heading;
    }

    public void setTargetPosition(double x, double y) {
        targetPos[0] = x;
        targetPos[1] = y;
        targetPos[2] = odo.getHeading();
    }

    public void setSpeedOnTargetPos(double x, double y, double yaw) {
        speedOnTargetPos[0] = x;
        speedOnTargetPos[1] = y;
        speedOnTargetPos[2] = yaw;
    }
    public void setSpeedOnTargetPos(double x, double y) {
        speedOnTargetPos[0] = x;
        speedOnTargetPos[1] = y;
        speedOnTargetPos[2] = 0;
    }

    public void setRunToPos() {
        mode = Modes.RUN_TO_POSITION;
    }

    public void setRunAntiLock() {
        mode = Modes.RUN_ANTILOCK_ONLY;
    }

    public Modes getMode() {
        return mode;
    }

}