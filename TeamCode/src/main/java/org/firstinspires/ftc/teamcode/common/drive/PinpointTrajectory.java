package org.firstinspires.ftc.teamcode.common.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;

//@Confg
public class PinpointTrajectory {
    //region Dashboard
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    //endregion

    //region Point Details
    public double[] startPoint;
    public double lt, lx, ly, ldx, ldy; //Last position
    public double t, x, y, dx, dy; //Present position
    public double heading, preH;
    public double delayTime;
    public double nowTime = 0;
    public Runnable fn; //Present function

    //Constants
    public static double TURN_SPEED = 0.4;
    public static double P_DRIVE_STEER_GAIN = 0.13;
    public static double P_DRIVE_TURN_GAIN = 0.17;
    public static double P_DRIVE_STRAIGHT_GAIN = 0.1;
    public static double motorVoltage = 12;

    public ElapsedTime runtime = new ElapsedTime();
    private final Robot robot;
    private double gotoX, gotoY, gotoH;

    public enum TrajectoryMode{
        DOT,
        VELOCITY
    }

    public enum Mode {
        TIMELINE,
        SEPARATED
    }

    private Mode pathMode = Mode.TIMELINE;
    //endregion

    //region Route Generator
    private double[] spline_fit(double x0, double dx0, double x1, double dx1) {
        double a = 2 * x0 + dx0 - 2 * x1 + dx1;
        double b = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
        double c = dx0;
        double d = x0;
        return new double[]{a, b, c, d};
    }

    private double spline_get(double[] spline, double u) {
        return spline[0] * u * u * u + spline[1] * u * u + spline[2] * u + spline[3];
    }
    //endregion

    //region Initialization
    public PinpointTrajectory(Robot robot){
        this.robot = robot;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odo.resetPosAndIMU();
    }

    public PinpointTrajectory reset(){
        robot.odo.resetPosAndIMU();
        return this;
    }
    //endregion

    //region Start Robot
    public PinpointTrajectory startMove(){
//        this.reset();
        return this.startMove(0, 0, 0, 0, () -> {});
    }

    public PinpointTrajectory startMove(Runnable fn){
        return this.startMove(0, 0, 0, 0, fn);
    }

    public PinpointTrajectory startMove(double n1, double n2, TrajectoryMode mode){
        return this.startMove(n1, n2, mode, () -> {});
    }

    public PinpointTrajectory startMove(double n1, double n2, TrajectoryMode mode, Runnable fn){
        if (mode == TrajectoryMode.DOT)
            return this.startMove(n1, n2, 0, 0, fn);
        return this.startMove(0, 0, n1, n2, fn);
    }

    public PinpointTrajectory startMove(double x, double y, double dx, double dy){
        return this.startMove(x, y, dx, dy, () -> {});
    }

    public PinpointTrajectory startMove(double[] point){
        return this.startMove(point[0], point[1], point[2], point[3]);
    }

    public PinpointTrajectory startMove(double x, double y, double dx, double dy, Runnable fn){
        //set start point
        startPoint = new double[]{x, y, dx, dy};
        this.t = 0;
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;
        this.robot.syncRun(fn);
        this.heading = getHeading();
        this.preH = getHeading();
        delayTime = 0;

        //start runtime
        runtime.reset();

        //FTCDashboard settings
        packet.fieldOverlay()
                .drawImage("/dash/into-the-deep.png", 0, 0, 144, 144, Math.toRadians(-90), 72, 72, false);

//        motorVoltage = robot.getVoltage();
        return this;
    }
    //endregion

    //region Control
    public PinpointTrajectory setMode(Mode mode){
        this.pathMode = mode;
        return this;
    }

    public PinpointTrajectory delay(double t){
        this.delayTime = 0;
        return this;
    }

    public PinpointTrajectory stopMotor(){
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    public PinpointTrajectory addPoint(double t, double x, double y, double dx, double dy){
        return this.addPoint(t, x, y, dx, dy, () -> {});
    }

    public PinpointTrajectory addPoint(double t, double x, double y, double dx, double dy, Runnable fn){
        return this.addPoint(t, x, y, dx, dy, this.heading, fn);
    }

    public PinpointTrajectory addPoint(double t, double x, double y, double dx, double dy, double heading){
        return this.addPoint(t, x, y, dx, dy, heading, () -> {});
    }

    public PinpointTrajectory addPoint(double t, double x, double y, double dx, double dy, double heading, Runnable fn){
        //record last point
//        t += delayTime;
        this.lt = this.t;
        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;

        if (this.pathMode == Mode.TIMELINE){
            this.t = t;
            //heading = Math.toRadians(heading);
        }
        else if (this.pathMode == Mode.SEPARATED){
            this.t += t;
        }

        this.t += this.delayTime;
        this.delayTime = 0;

        //present point
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;

        double TURN_GAIN;
        if (heading == this.heading) TURN_GAIN = P_DRIVE_STEER_GAIN;
        else TURN_GAIN = P_DRIVE_TURN_GAIN;



        //main movement methods
        double x_error, y_error, turnSpeed, motorPowerGain;
        double now = runtime.seconds();

        while (now <= this.t && now > this.lt && robot.opMode.opModeIsActive()){
            //update current pose
            robot.odo.update();

            double curH = getHeading();

            motorPowerGain = Math.abs(motorVoltage / robot.getVoltage());

            //get current time
            now = runtime.seconds();

            //calculate going position
            gotoX = spline_get(
                    spline_fit(this.lx - startPoint[0], this.ldx,
                            this.x - startPoint[0], this.dx),
                    (now - this.lt) / (this.t - this.lt)
            );
            gotoY = spline_get(
                    spline_fit(this.ly - startPoint[1], this.ldy,
                            this.y - startPoint[1], this.dy),
                    (now - this.lt) / (this.t - this.lt)
            );
            gotoH = spline_get(
                    spline_fit(this.heading, 0, heading, 0),
                    (now - this.lt) / (this.t - this.lt)
            );

            //calculate errors
            double h = Math.abs(preH - curH) < 300 ?
                    curH
                    : curH + 360 * (preH > 0 ? 1 : -1);
            x_error = gotoX - getX();
            y_error = gotoY - getY();
            turnSpeed = Range.clip((gotoH - h) * TURN_SPEED * TURN_GAIN, -1, 1);
            preH = h;

            //drive robot
            if (!Globals.DEBUG) robot.odoDrivetrain.driveRobotFieldCentric(
                    x_error * P_DRIVE_STRAIGHT_GAIN * motorPowerGain,
                    -y_error * P_DRIVE_STRAIGHT_GAIN * motorPowerGain,
                    -turnSpeed * motorPowerGain
            );

            //FTC Dashboard
            packet.fieldOverlay().setRotation(-Math.toRadians(90)).setTranslation(4 + startPoint[1], -6 * 12 + 7 + startPoint[0]).setStroke("blue").setStrokeWidth(1).strokeRect(gotoX - 8.5, gotoY - 9, 17, 18);
            dashboard.sendTelemetryPacket(packet);
            if (Globals.DEBUG) robot.sleep(200);


            //telemetry datas
            robot.telemetry.addData("gotoX", gotoX);
            robot.telemetry.addData("gotoY", gotoY);
            robot.telemetry.addData("preH", preH);
            robot.telemetry.addData("gotoH", gotoH);
            robot.telemetry.addData("turnSpeed", turnSpeed);
            robot.telemetry.addLine();

            robot.telemetry.addData("X_POS", getX());
            robot.telemetry.addData("Y_POS", getY());
            robot.telemetry.addData("Heading", getHeading());
            robot.telemetry.addData("TURN_GAIN", TURN_GAIN);
            robot.telemetry.addData("Voltage", robot.getVoltage());
            robot.telemetry.addData("MOTOR_GAIN", motorPowerGain);
            robot.telemetry.addData("Odo", robot.odo.getPosition().toString());
            robot.telemetry.addLine();

            robot.telemetry.addData("RunTime", now);
            robot.telemetry.addData("ToTime", this.t);

            robot.telemetry.update();
        }

        this.heading = heading;

        //run function
        if ((!Globals.DEBUG || Globals.DEBUG_RUN_FUNC) && !Globals.DO_NOT_RUN_FUNC) this.robot.syncRun(fn);

        return this;
    }

    public PinpointTrajectory addVelocity(double dx, double dy){
        return this.addPoint(this.t, this.x, this.y, dx, dy);
    }

    public PinpointTrajectory addFunc(Runnable fn){
        return this.addPoint(this.t, this.x, this.y, this.dx, this.dy, fn);
    }

    public PinpointTrajectory addTime(double toTime){
        return this.addTime(toTime, () -> {});
    }

    public PinpointTrajectory addTime(double toTime, Runnable fn){
        return this.addPoint(toTime, this.x, this.y, this.dx, this.dy, fn);
    }

    public PinpointTrajectory addHeading(double t, double heading){
        return this.addHeading(t, heading, () -> {});
    }

    public PinpointTrajectory addHeading(double t, double heading, Runnable fn){
        return this.addPoint(t, this.x, this.y, this.dx, this.dy, heading, fn);
    }
    //endregion

    //region Get Data
    public double getX(){
        return this.getX(DistanceUnit.INCH);
    }

    public double getX(DistanceUnit unit){
        return robot.odo.getPosition().getX(unit);
    }

    public double getY(){
        return this.getY(DistanceUnit.INCH);
    }

    public double getY(DistanceUnit unit){
        return robot.odo.getPosition().getY(unit);
    }

    public double getHeading(){
        return this.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading(AngleUnit unit){
        return robot.odo.getPosition().getHeading(unit);
    }

    public Pose2D getPosition(){
        return robot.odo.getPosition();
    }

    public double[] getStartPoint(){
        return this.startPoint;
    }

    public double[] getGotoPoint(){
        return new double[]{this.x, this.y, this.dx, this.dy};
    }

    public double[] getCurrentGotoPoint(){
        return new double[]{gotoX, gotoY};
    }

    public double[] getPreviousPoint(){
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }
    //endregion
}