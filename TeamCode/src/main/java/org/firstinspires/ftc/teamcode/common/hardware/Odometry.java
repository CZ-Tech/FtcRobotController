package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.teamcode.common.Globals.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.common.Globals.LATERAL_DISTANCE;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Matrix;
import org.firstinspires.ftc.teamcode.common.util.MatrixEx;

public enum Odometry {
    INSTANCE;
    private final Robot robot =  Robot.INSTANCE;
    private final Matrix matrix = Matrix.INSTANCE;
    private final MatrixEx k1 = new MatrixEx(new double[][] {{0,0}, {0,0}});
    private final MatrixEx k2 = new MatrixEx(new double[][] {{0,0}, {0,0}});
    private final MatrixEx temp = new MatrixEx(new double[][] {{0}, {0}});
    private final MatrixEx result = new MatrixEx(new double[][] {{0}, {0}});


    private DcMotorEx encoderLeft = null;
    private DcMotorEx encoderRight = null;
    private DcMotorEx encoderCenter = null;

    private double prevLeftEncoder, prevRightEncoder, prevCenterEncoder, phi, delta_middle, delta_perp_pos;
    public double delta_x, delta_y, X_Pos, Y_Pos, heading;
    private double[][] matrix1;
    private double[][] matrix2;
//    private double[] result;

//  Control Hub 0 -> Center
//  Control Hub 1 -> Left
//  Control Hub 2 -> Right
    Odometry() {
        encoderCenter = robot.hardwareMap.get(DcMotorEx.class, "cencoder");
        encoderLeft = robot.hardwareMap.get(DcMotorEx.class, "lencoder");
        encoderRight = robot.hardwareMap.get(DcMotorEx.class, "rencoder");
        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevCenterEncoder = 0;
    }
    public void resetPos() {
        X_Pos = 0;
        Y_Pos = 0;
        delta_x = 0;
        delta_y = 0;
    }

    //使用时，把update()单独开一个线程搞。
    public void update() {
        phi = (getDeltaLeftPostion() + getDeltaRightPostion()) / LATERAL_DISTANCE;
        delta_middle = (getDeltaLeftPostion() + getDeltaRightPostion()) / 2;
        delta_perp_pos = getDeltaCenterPostion() - FORWARD_OFFSET * phi; //imu没法返回一个double，暂时用惰轮直接计算。

        //TODO: 待测试。
        k2.set(new double[][] {{Math.sin(phi) / phi,    (Math.cos(phi) - 1) / phi},
                              {(1-Math.cos(phi)) / phi,  Math.sin(phi) / phi     }});
        k1.set(new double[][] {{Math.cos(heading), -Math.sin(heading)},
                              {Math.sin(heading), Math.cos(heading)}});
        temp.set(new double[][] {{delta_middle}, {delta_perp_pos}});
        if(phi != 0) {
//            result = matrix.times(matrix2, matrix.times(matrix1, new double[]{delta_middle, delta_perp_pos}));
            result.set(k1.multiply(k2.multiply(temp)));
            delta_x = result.data[0][0];
            delta_y = result.data[1][0];
        } else {
            result.set(k1.multiply(temp));
            delta_x = result.data[0][0];
            delta_y = result.data[1][0];
        }

        heading += phi;
        X_Pos += delta_x;
        Y_Pos += delta_y;

        prevCenterEncoder = encoderCenter.getCurrentPosition();
        prevLeftEncoder = encoderLeft.getCurrentPosition();
        prevRightEncoder = encoderRight.getCurrentPosition();
    }

    public double getDeltaLeftPostion() {
        return encoderLeft.getCurrentPosition() - this.prevLeftEncoder;
    }

    public double getDeltaRightPostion() {
        return encoderRight.getCurrentPosition() - this.prevRightEncoder;
    }

    public double getDeltaCenterPostion() {
        return encoderCenter.getCurrentPosition() - this.prevCenterEncoder;
    }

    public AngularVelocity getDeltaTheta() {
        return robot.imu.getRobotAngularVelocity(AngleUnit.RADIANS);
    }




}
