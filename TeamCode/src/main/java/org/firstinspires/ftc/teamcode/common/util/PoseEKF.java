package org.firstinspires.ftc.teamcode.common.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A standalone Extended Kalman Filter for FTC field-centric pose estimation.
 * Uses a Producer-Consumer architecture and includes a custom lightweight Matrix class.
 */
public class PoseEKF {

    // --- 1. Lightweight Custom Matrix Class ---
    public static class Matrix {
        public final int rows;
        public final int cols;
        public final double[][] data;

        public Matrix(int rows, int cols) {
            this.rows = rows;
            this.cols = cols;
            this.data = new double[rows][cols];
        }

        public void set(int r, int c, double val) { data[r][c] = val; }
        public double get(int r, int c) { return data[r][c]; }

        public static Matrix identity(int size) {
            Matrix m = new Matrix(size, size);
            for (int i = 0; i < size; i++) m.set(i, i, 1.0);
            return m;
        }

        public static Matrix diag(double... values) {
            Matrix m = new Matrix(values.length, values.length);
            for (int i = 0; i < values.length; i++) m.set(i, i, values[i]);
            return m;
        }

        public Matrix plus(Matrix other) {
            Matrix result = new Matrix(rows, cols);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    result.set(r, c, this.get(r, c) + other.get(r, c));
            return result;
        }

        public Matrix minus(Matrix other) {
            Matrix result = new Matrix(rows, cols);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    result.set(r, c, this.get(r, c) - other.get(r, c));
            return result;
        }

        public Matrix mult(Matrix other) {
            if (this.cols != other.rows) throw new IllegalArgumentException("Dimension mismatch for multiplication");
            Matrix result = new Matrix(this.rows, other.cols);
            for (int r = 0; r < this.rows; r++) {
                for (int c = 0; c < other.cols; c++) {
                    double sum = 0;
                    for (int k = 0; k < this.cols; k++) {
                        sum += this.get(r, k) * other.get(k, c);
                    }
                    result.set(r, c, sum);
                }
            }
            return result;
        }

        public Matrix transpose() {
            Matrix result = new Matrix(cols, rows);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    result.set(c, r, this.get(r, c));
            return result;
        }

        public Matrix scale(double scalar) {
            Matrix result = new Matrix(rows, cols);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    result.set(r, c, this.get(r, c) * scalar);
            return result;
        }

        // Hardcoded 3x3 Inverse for extreme performance and zero dependencies
        public Matrix invert3x3() {
            if (rows != 3 || cols != 3) throw new UnsupportedOperationException("Only 3x3 matrices supported for this invert function");

            double a = data[0][0], b = data[0][1], c = data[0][2];
            double d = data[1][0], e = data[1][1], f = data[1][2];
            double g = data[2][0], h = data[2][1], i = data[2][2];

            double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
            if (Math.abs(det) < 1e-9) throw new ArithmeticException("Matrix is singular or near-singular");

            Matrix inv = new Matrix(3, 3);
            inv.set(0, 0, (e*i - f*h) / det);
            inv.set(0, 1, (c*h - b*i) / det);
            inv.set(0, 2, (b*f - c*e) / det);
            inv.set(1, 0, (f*g - d*i) / det);
            inv.set(1, 1, (a*i - c*g) / det);
            inv.set(1, 2, (c*d - a*f) / det);
            inv.set(2, 0, (d*h - e*g) / det);
            inv.set(2, 1, (b*g - a*h) / det);
            inv.set(2, 2, (a*e - b*d) / det);
            return inv;
        }
    }


    // --- 3. EKF Core Variables ---
    private Matrix state; // [x, y, theta]^T
    private Matrix P;     // Covariance matrix
    private Matrix Q;     // Process noise (Trust in Odometry)
    private Matrix R;     // Measurement noise (Trust in Vision)

    // Functional Interfaces
    private Supplier<Pose2D> odometryDeltaSupplier;
    private Supplier<Optional<Pose2D>> visionPoseSupplier;
    private Consumer<Pose2D> estimatedPoseConsumer;

    public PoseEKF(Pose2D initialPose) {
        state = new Matrix(3, 1);
        state.set(0, 0, initialPose.getX(DistanceUnit.METER));
        state.set(1, 0, initialPose.getY(DistanceUnit.METER));
        state.set(2, 0, initialPose.getHeading(AngleUnit.RADIANS));

        // 初始化协方差
        P = Matrix.identity(3).scale(0.1);

        // 过程噪声 (Q) 和 测量噪声 (R) 需要根据你的实际硬件调整
        Q = Matrix.diag(0.05, 0.05, 0.01);
        R = Matrix.diag(0.1, 0.1, 99999.0);
    }

    // 依赖注入
    public void setOdometrySupplier(Supplier<Pose2D> supplier) { this.odometryDeltaSupplier = supplier; }
    public void setVisionSupplier(Supplier<Optional<Pose2D>> supplier) { this.visionPoseSupplier = supplier; }
    public void setPoseConsumer(Consumer<Pose2D> consumer) { this.estimatedPoseConsumer = consumer; }

    /**
     * EKF 主循环
     */
    public void update() {
        if (odometryDeltaSupplier == null || estimatedPoseConsumer == null) {
            throw new IllegalStateException("Odometry supplier and Pose consumer must be set.");
        }

        // 1. 预测 (Prediction)
        Pose2D odomDelta = odometryDeltaSupplier.get();
        predict(odomDelta);

        // 2. 更新 (Update)
        if (visionPoseSupplier != null) {
            Optional<Pose2D> visionMeasurement = visionPoseSupplier.get();
            visionMeasurement.ifPresent(this::correct);
        }

        // 3. 消费 (Consume)
        Pose2D currentEstimate = new Pose2D(DistanceUnit.METER, state.get(0, 0), state.get(1, 0), AngleUnit.RADIANS, state.get(2, 0));
        estimatedPoseConsumer.accept(currentEstimate);
    }

    private void predict(Pose2D delta) {
        double currentHeading = state.get(2, 0);
        double cosTh = Math.cos(currentHeading);
        double sinTh = Math.sin(currentHeading);

        // 更新状态方程 (Non-linear)
        double nextX = state.get(0, 0) + delta.getX(DistanceUnit.METER) * cosTh - delta.getY(DistanceUnit.METER) * sinTh;
        double nextY = state.get(1, 0) + delta.getX(DistanceUnit.METER) * sinTh + delta.getY(DistanceUnit.METER) * cosTh;
        double nextHeading = state.get(2, 0) + delta.getHeading(AngleUnit.RADIANS);

        state.set(0, 0, nextX);
        state.set(1, 0, nextY);
        state.set(2, 0, nextHeading);

        // 过程雅可比矩阵 (Jacobian F)
        Matrix F = Matrix.identity(3);
        F.set(0, 2, -delta.getX(DistanceUnit.METER) * sinTh - delta.getY(DistanceUnit.METER) * cosTh);
        F.set(1, 2,  delta.getX(DistanceUnit.METER) * cosTh - delta.getY(DistanceUnit.METER) * sinTh);

        // 更新协方差: P = F * P * F^T + Q
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    private void correct(Pose2D measurement) {
        Matrix z = new Matrix(3, 1);
        z.set(0, 0, measurement.getX(DistanceUnit.METER));
        z.set(1, 0, measurement.getY(DistanceUnit.METER));
        z.set(2, 0, measurement.getHeading(AngleUnit.RADIANS));

        // 测量雅可比矩阵 (H 是 3x3 单位矩阵)
        Matrix H = Matrix.identity(3);

        // S = H * P * H^T + R
        Matrix S = H.mult(P).mult(H.transpose()).plus(R);

        // K = P * H^T * S^-1
        Matrix K = P.mult(H.transpose()).mult(S.invert3x3());

        // y = z - H * x
        Matrix y = z.minus(H.mult(state));

        // 航向角越界处理 (确保误差在 -pi 到 pi 之间)
        double headingDiff = y.get(2, 0);
        while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
        while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;
        y.set(2, 0, headingDiff);

        // 状态更新: x = x + K * y
        state = state.plus(K.mult(y));

        // 协方差更新: P = (I - K * H) * P
        Matrix I = Matrix.identity(3);
        P = I.minus(K.mult(H)).mult(P);
    }

    /**
     * 强制重置当前机器人的绝对场地坐标。
     * 常用于 Autonomous 初始化阶段，或当检测到极高置信度的绝对定位标志时。
     */
    public void resetPose(double x, double y, double heading) {
        state.set(0, 0, x);
        state.set(1, 0, y);
        state.set(2, 0, heading);
        // 重置状态的同时，将协方差矩阵 P 重置为初始的小不确定性状态
        P = Matrix.identity(3).scale(0.1);
    }

    // 重载方法，支持传入 Pose2D 对象
    public void resetPose(Pose2D pose) {
        resetPose(pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * 以基础数组的形式返回当前的坐标估计值。
     * @return double[] {x, y, heading}，其中 heading 的单位是弧度。
     */
    public double[] getEstimatedPoseArray() {
        return new double[] {
                state.get(0, 0),
                state.get(1, 0),
                state.get(2, 0)
        };
    }
}
