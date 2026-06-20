package org.firstinspires.ftc.teamcode.common.util;

import static org.firstinspires.ftc.teamcode.common.Globals.PID_THREAD_Hz;

import org.firstinspires.ftc.teamcode.common.frames.TaskLoopFrame;

import java.util.Arrays;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;


public class PIDController extends TaskLoopFrame {
    private double P;
    private double I;
    private double D;

    // 状态变量
    private double integral = 0.0;
    private double previousError = 0.0;

    // 积分限幅（Anti-windup）参数
    private double maxIntegral = Double.MAX_VALUE;
    private double minIntegral = -Double.MAX_VALUE;

    private volatile double setpoint;
    private final DoubleSupplier pvProvider;
    private final DoubleSupplier timeProvider;
    private final DoubleConsumer outputConsumer;
    private final DoubleBinaryOperator feedforwardProvider;


    // 用于计算速度和加速度的状态变量
    private double previousPV = 0.0;
    private double currentAcceleration = 0.0;
    private boolean isFirstRun = true;

    // 速度滤波
    public static int VelFilterWindowSize = 4;
    public double[] shootVelWindow = new double[VelFilterWindowSize];
    private int currentShootVelWindowIndex = 0;
    private double filteredSpeed = 0.0;
    private boolean filterInitialized = false;

    private volatile boolean shouldMotorRun = false;

    private double lastOutput = 0;

    public static class DeltaNanoProvider implements DoubleSupplier {
        private double lastNano = System.nanoTime();
        @Override
        public double getAsDouble() {
            double now = System.nanoTime();
            double delta = now - lastNano;
            lastNano = now;
            return delta * 1e-9;
        }
    }

    public void setPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
    }
    
    /**
     * @param PIDName          线程名，会关联logcat的日志输出消息
     * @param pvProvider       当前实际值提供者 (Process Variable)
     * @param timeProvider     时间间隔提供者
     * @param outputConsumer   计算结果输出消费者
     */
    public PIDController(String PIDName, double P, double I, double D,
                         DoubleSupplier pvProvider,
                         DoubleSupplier timeProvider,
                         DoubleConsumer outputConsumer,
                         DoubleBinaryOperator feedforwardProvider) {
        super(PID_THREAD_Hz, PIDName);
        this.P = P;
        this.I = I;
        this.D = D;
        this.pvProvider = pvProvider;
        this.timeProvider = timeProvider;
        this.outputConsumer = outputConsumer;
        this.feedforwardProvider = feedforwardProvider;
        reset();
    }

    /**
     * 若pid线程尚未启动，本方法会自动启动
     * @param target 目标值
     */
    public void setTarget(double target) {
        setpoint = target;
        this.start();
        startMotor();
    }

    public double getTarget() {
        return setpoint;
    }

    /**
     * @param PIDName 线程名，会关联logcat的日志输出消息
     * @param pvProvider       当前实际值提供者 (Process Variable)
     * @param outputConsumer   计算结果输出消费者
     * @param feedforwardProvider 前馈控制输出，传入的第一个参数是目标位置，第二个是上次输出的功率（含上次前馈）
     */
    public PIDController(String PIDName, double P, double I, double D,
                         DoubleSupplier pvProvider,
                         DoubleConsumer outputConsumer,
                         DoubleBinaryOperator feedforwardProvider) {
        this(PIDName, P, I, D, pvProvider, new DeltaNanoProvider(), outputConsumer, feedforwardProvider);
    }

    /**
     * 设置积分限幅，防止积分饱和（Anti-windup）
     */
    public void setIntegralLimits(double min, double max) {
        this.minIntegral = min;
        this.maxIntegral = max;
    }

    /**
     * 重置PID状态（通常在系统重启或目标值发生大幅跳变时调用）
     */
    public void reset() {
        this.integral = 0.0;
        this.previousError = 0.0;
        this.isFirstRun = true;
        this.filterInitialized = false;
    }

    /**
     * PID 计算核心逻辑
     */
    public void compute() {

        double dt = timeProvider.getAsDouble();

        updateFilter(pvProvider.getAsDouble(), dt);

        // 如果 dt 超过设定刷新时间的十倍，说明线程刚恢复或发生了严重卡顿
        if (dt > targetUpdateMs * 10 / 1000.0) { // targetUpdateMs 是毫秒，dt 是秒
            dt = 0; // 丢弃这一帧的异常时间，防止积分爆炸
            isFirstRun = true; // 强制重置状态防止微分项突变
        }

        // 防止除以0或负时间导致的异常
        if (dt <= 0) {
            return;
        }

        double processVariable = getVelocity();

        // 1. 计算当前误差
        double error = setpoint - processVariable;

        if (isFirstRun) {
            previousPV = processVariable;
            previousError = error; // 同步误差
            currentAcceleration = 0.0;
            isFirstRun = false;
        } else {
            currentAcceleration = (processVariable - previousPV) / dt;
            previousPV = processVariable;
        }

        // 2. 比例项 (Proportional)
        double pTerm = P * error;

        // 3. 积分项 (Integral)
        integral += error * dt;
        // 积分限幅
        integral = Math.max(minIntegral, Math.min(maxIntegral, integral));
        double iTerm = I * integral;

        // 4. 微分项 (Derivative)
        double derivative = (error - previousError) / dt;
        double dTerm = D * derivative;

        // 5. 计算总输出
        lastOutput = pTerm + iTerm + dTerm + feedforwardProvider.applyAsDouble(setpoint, lastOutput);

        // 6. 记录本次误差，供下次微分计算使用
        previousError = error;

        // 只有当目标值为 0 且误差极小时才主动停止电机，而不是简单按输出大小判断
        if (Math.abs(setpoint) < 1.0 && Math.abs(error) < 5.0) {
            lastOutput = 0.0;
            outputConsumer.accept(0.0);
            return;
        }

        // 7. 将结果交由消费者执行
        outputConsumer.accept(lastOutput);
    }


    /**
     * 用于更新逻辑的方法
     * 调用start()后会尽可能以指定速率循环调用该方法
     * 但是没有滞后补偿机制
     */
    @Override
    public void update() {
        if (shouldMotorRun) compute();
        else outputConsumer.accept(0.0);
    }

    private void updateFilter(double rawVel, double dt) {
        // 处理长时间停止后的重置逻辑 (假设阈值 0.5 秒)
        if (dt > 0.5) {
            Arrays.fill(shootVelWindow, rawVel);
            filteredSpeed = rawVel;
            filterInitialized = true;
            return;
        }

        // 首次启动：用当前原始值填满窗口，避免零初值导致启动时速度被低估
        if (!filterInitialized) {
            Arrays.fill(shootVelWindow, rawVel);
            filteredSpeed = rawVel;
            filterInitialized = true;
            return;
        }

        // 更新循环队列
        shootVelWindow[currentShootVelWindowIndex] = rawVel;
        currentShootVelWindowIndex = (currentShootVelWindowIndex + 1) % shootVelWindow.length;

        // 计算平均值
        double totalVel = 0.0;
        for (double v : shootVelWindow) {
            totalVel += v;
        }
        filteredSpeed = totalVel / shootVelWindow.length;
    }

    /**
     * 默认在stop时会把电机功率设为0
     */
    @Override
    public void onStop() {
        outputConsumer.accept(0.0);
        shouldMotorRun = false;
    }

    /**
     * 这个方法会使电机保持当前功率继续运行但不再计算PID
     */
    public void stopPIDOnly() {
        super.stop();
    }

    public void breakMotor() {
        setpoint = 0;
    }

    @Override
    public void onStart() {
        reset();
        shouldMotorRun = true;
    }

    public void startMotor() {
        shouldMotorRun = true;
    }

    public void stopMotor() {
        shouldMotorRun = false;
    }

    /**
     * 获取最近一次计算的输出功率
     */
    public double getCurrentOutput() {
        return lastOutput;
    }

    /**
     * 获取当前电机的速度
     * 单位：[PV单位] / 秒
     */
    public double getVelocity() {
        return filteredSpeed;
    }

    /**
     * 获取当前电机的加速度 (Process Variable 的二阶导数)
     * 单位：[PV单位] / 秒^2
     */
    public double getAcceleration() {
        return currentAcceleration;
    }

}
