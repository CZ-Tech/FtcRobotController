package org.firstinspires.ftc.teamcode.common.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.frames.TaskLoopFrame;


public class SplineTracker {
    public static boolean DEBUG_RUN_FUNC = true;
    public static boolean DO_NOT_RUN_FUNC = false;

    //region Point Details
    public double[] startPoint;
    public double lx, ly, ldx, ldy;
    public double x, y, dx, dy;
    public double heading, preH;
    public Runnable fn;

    // Tunable gains
    public static double K_DISTANCE = 0.05;
    public static double K_VELOCITY = 0.05;
    public static double K_CROSS_TRACK = 0.1;
    public static double K_HEADING = 0.03;
    public static int SAMPLE_COUNT = 20;
    public static int NEWTON_ITERS = 3;
    public static double END_U_THRESHOLD = 0.98;
    public static double END_POS_THRESHOLD = 1.0;
    public static double END_HEADING_THRESHOLD = 2.0;
    public static double MAX_SEGMENT_TIMEOUT = 5.0;

    public static double TURN_DEAD_AREA = 1;
    public static double motorVoltage = 12;
    public static double VOLTAGE_COMP_WEIGHT = 0.1;

    public ElapsedTime runtime = new ElapsedTime();
    private final Robot robot;
    private double gotoX, gotoY, gotoH;
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

    private double splineDerivative(double[] spline, double u) {
        return 3 * spline[0] * u * u + 2 * spline[1] * u + spline[2];
    }

    private double splineSecondDerivative(double[] spline, double u) {
        return 6 * spline[0] * u + 2 * spline[1];
    }

    private double clamp01(double u) {
        if (u < 0) return 0;
        if (u > 1) return 1;
        return u;
    }

    private double findClosestU(double[] splineX, double[] splineY, double rx, double ry) {
        // Step 1: Sample along the spline to find best initial u
        double bestU = 0;
        double bestDist = Double.MAX_VALUE;
        int samples = SAMPLE_COUNT;
        for (int i = 0; i <= samples; i++) {
            double u = (double) i / samples;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            double dx = sx - rx;
            double dy = sy - ry;
            double dist = dx * dx + dy * dy;
            if (dist < bestDist) {
                bestDist = dist;
                bestU = u;
            }
        }

        // Step 2: Newton refinement
        for (int iter = 0; iter < NEWTON_ITERS; iter++) {
            double u = bestU;
            double sx = spline_get(splineX, u);
            double sy = spline_get(splineY, u);
            double sdx = splineDerivative(splineX, u);
            double sdy = splineDerivative(splineY, u);
            double sddx = splineSecondDerivative(splineX, u);
            double sddy = splineSecondDerivative(splineY, u);

            double ex = sx - rx;
            double ey = sy - ry;

            // f'(u) = 2*(sx-rx)*sdx + 2*(sy-ry)*sdy
            double fprime = 2 * ex * sdx + 2 * ey * sdy;
            // f''(u) = 2*sdx*sdx + 2*ex*sddx + 2*sdy*sdy + 2*ey*sddy
            double fsecond = 2 * sdx * sdx + 2 * ex * sddx + 2 * sdy * sdy + 2 * ey * sddy;

            if (Math.abs(fsecond) < 1e-9) break;

            double uNew = u - fprime / fsecond;
            uNew = clamp01(uNew);

            if (Math.abs(uNew - u) < 1e-6) {
                bestU = uNew;
                break;
            }
            bestU = uNew;
        }

        return bestU;
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }
    //endregion

    //region Initialization
    public SplineTracker(Robot robot) {
        this.robot = robot;
        robot.odoDrivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odo.resetPosAndIMU();
    }

    public Robot getRobot() {
        return robot;
    }

    public SplineTracker reset() {
        robot.odo.resetPosAndIMU();
        return this;
    }

    public SplineTracker setPose(Pose2D pose) {
        robot.odo.setGlobalPose(pose);
        return this;
    }
    //endregion

    //region Start Robot
    public SplineTracker startMove() {
        return this.startMove(0, 0, 0, 0, () -> {});
    }

    public SplineTracker startMove(Runnable fn) {
        return this.startMove(0, 0, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y) {
        return this.startMove(x, y, 0, 0, () -> {});
    }

    public SplineTracker startMove(double x, double y, Runnable fn) {
        return this.startMove(x, y, 0, 0, fn);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy) {
        return this.startMove(x, y, dx, dy, () -> {});
    }

    public SplineTracker startMove(double[] point) {
        return this.startMove(point[0], point[1], point[2], point[3]);
    }

    public SplineTracker startMove(double x, double y, double dx, double dy, Runnable fn) {
        startPoint = new double[]{x, y, dx, dy};
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;
        TaskLoopFrame.runOnce(fn);
        this.heading = getHeading();
        this.preH = getHeading();

        runtime.reset();

        return this;
    }
    //endregion

    //region Control
    public SplineTracker stopMotor() {
        robot.odoDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.odoDrivetrain.stopMotor();
        return this;
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy) {
        return this.addPoint(x, y, dx, dy, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, Runnable fn) {
        return this.addPoint(x, y, dx, dy, this.heading, fn);
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading) {
        return this.addPoint(x, y, dx, dy, heading, () -> {});
    }

    public SplineTracker addPoint(double x, double y, double dx, double dy, double heading, Runnable fn) {
        // Record last point (start of this spline segment)
        this.lx = this.x;
        this.ly = this.y;
        this.ldx = this.dx;
        this.ldy = this.dy;

        // Set target
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
        this.fn = fn;

        // Build position splines
        double[] splineX = spline_fit(this.lx, this.ldx, this.x, this.dx);
        double[] splineY = spline_fit(this.ly, this.ldy, this.y, this.dy);

        // Unwrap target heading to shortest path from current heading
        double unwrappedHeading = heading;
        double hDiff = heading - this.heading;
        while (hDiff > 180) { unwrappedHeading -= 360; hDiff -= 360; }
        while (hDiff <= -180) { unwrappedHeading += 360; hDiff += 360; }

        double[] splineH = spline_fit(this.heading, 0, unwrappedHeading, 0);

        double segStartTime = runtime.seconds();

        while (robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double rx = getX();
            double ry = getY();

            double rawMotorPowerGain = Math.abs(motorVoltage / robot.getVoltage());
            double motorPowerGain = 1.0 + (rawMotorPowerGain - 1.0) * VOLTAGE_COMP_WEIGHT;

            // Find closest point on spline
            double uStar = findClosestU(splineX, splineY, rx, ry);

            // Position, tangent, normal at u*
            double px = spline_get(splineX, uStar);
            double py = spline_get(splineY, uStar);
            double vx = splineDerivative(splineX, uStar);
            double vy = splineDerivative(splineY, uStar);
            double mag = Math.hypot(vx, vy);

            double tx, ty;
            if (mag > 1e-6) {
                tx = vx / mag;
                ty = vy / mag;
            } else {
                tx = Math.cos(Math.toRadians(curH));
                ty = Math.sin(Math.toRadians(curH));
            }
            double nx = -ty;
            double ny = tx;

            // Error decomposition
            double ex = rx - px;
            double ey = ry - py;
            double eCross = ex * nx + ey * ny;

            // Tangent power: distance to target + target velocity feedforward
            double distToTarget = Math.hypot(this.x - rx, this.y - ry);
            double targetSpeed = Math.hypot(this.dx, this.dy);
            double tanPower = (distToTarget * K_DISTANCE + targetSpeed * K_VELOCITY) * motorPowerGain;
            // Normal power: correct lateral deviation from path
            double normPower = -eCross * K_CROSS_TRACK * motorPowerGain;

            double vxF = tanPower * tx + normPower * nx;
            double vyF = tanPower * ty + normPower * ny;

            // Heading control
            double desiredH = spline_get(splineH, uStar);
            double headingError = angleDiff(desiredH, curH);
            double yawPower = headingError * K_HEADING * motorPowerGain;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;

            vxF = Range.clip(vxF, -1, 1);
            vyF = Range.clip(vyF, -1, 1);
            yawPower = Range.clip(yawPower, -1, 1);

            // Drive robot
            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(
                        vxF, -vyF, -yawPower
                );

            // FTC Dashboard
            if (Globals.DEBUG) robot.sleep(200);

            // Telemetry
            robot.telemetry.addData("u*", uStar);
            robot.telemetry.addData("gotoX", px);
            robot.telemetry.addData("gotoY", py);
            robot.telemetry.addData("desiredH", desiredH);
            robot.telemetry.addData("eCross", eCross);
            robot.telemetry.addData("tanPower", tanPower);
            robot.telemetry.addData("normPower", normPower);
            robot.telemetry.addLine();

            robot.telemetry.addData("X_POS", rx);
            robot.telemetry.addData("Y_POS", ry);
            robot.telemetry.addData("Heading", curH);
            robot.telemetry.addData("Voltage", robot.getVoltage());
            robot.telemetry.addData("MOTOR_GAIN", motorPowerGain);
            robot.telemetry.addData("Odo", robot.odo.getPosition().toString());

            // Store for accessors
            gotoX = px;
            gotoY = py;
            gotoH = desiredH;

            // Exit condition: reached end of path segment
            boolean done = uStar >= END_U_THRESHOLD
                    && Math.abs(eCross) < END_POS_THRESHOLD
                    && Math.abs(headingError) < END_HEADING_THRESHOLD;
            boolean timedOut = runtime.seconds() - segStartTime > MAX_SEGMENT_TIMEOUT;

            if (done || timedOut) break;
        }

        this.heading = unwrappedHeading;
        this.preH = unwrappedHeading;

        // Run function at end of segment
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) TaskLoopFrame.runOnce(fn);

        return this;
    }

    public SplineTracker addVelocity(double dx, double dy) {
        this.ldx = this.dx;
        this.ldy = this.dy;
        this.dx = dx;
        this.dy = dy;
        return this;
    }

    public SplineTracker addFunc(Runnable fn) {
        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC)
            TaskLoopFrame.runOnce(fn);
        return this;
    }

    public SplineTracker addTime(double seconds) {
        return this.addTime(seconds, () -> {});
    }

    public SplineTracker addTime(double seconds, Runnable fn) {
        double startWait = runtime.seconds();

        while (runtime.seconds() - startWait < seconds && robot.opMode.opModeIsActive()) {
            robot.odo.update();

            double curH = getHeading();
            double headingError = angleDiff(this.heading, curH);
            double yawPower = headingError * K_HEADING;
            if (Math.abs(headingError) < TURN_DEAD_AREA) yawPower = 0;
            yawPower = Range.clip(yawPower, -1, 1);

            if (!Globals.DEBUG)
                robot.odoDrivetrain.driveRobotFieldCentric(0, 0, -yawPower);
            else
                robot.sleep(200);

            robot.telemetry.addData("addTime", seconds);
            robot.telemetry.addData("remaining", String.format("%.2f", seconds - (runtime.seconds() - startWait)));
            robot.telemetry.addLine();
        }

        if ((!Globals.DEBUG || DEBUG_RUN_FUNC) && !DO_NOT_RUN_FUNC) TaskLoopFrame.runOnce(fn);

        return this;
    }

    public SplineTracker addHeading(double heading) {
        return this.addHeading(heading, () -> {});
    }

    public SplineTracker addHeading(double heading, Runnable fn) {
        return this.addPoint(this.x, this.y, this.dx, this.dy, heading, fn);
    }
    //endregion

    //region Get Data
    public double getX() {
        return this.getX(DistanceUnit.INCH);
    }

    public double getX(DistanceUnit unit) {
        return robot.odo.getPosition().getX(unit);
    }

    public double getY() {
        return this.getY(DistanceUnit.INCH);
    }

    public double getY(DistanceUnit unit) {
        return robot.odo.getPosition().getY(unit);
    }

    public double getHeading() {
        return this.getHeading(AngleUnit.DEGREES);
    }

    public double getHeading(AngleUnit unit) {
        return robot.odo.getPosition().getHeading(unit);
    }

    public Pose2D getPosition() {
        return robot.odo.getPosition();
    }

    public double[] getStartPoint() {
        return this.startPoint;
    }

    public double[] getGotoPoint() {
        return new double[]{this.x, this.y, this.dx, this.dy};
    }

    public double[] getCurrentGotoPoint() {
        return new double[]{gotoX, gotoY};
    }

    public double[] getPreviousPoint() {
        return new double[]{this.lx, this.ly, this.ldx, this.ldy};
    }
    //endregion
}
