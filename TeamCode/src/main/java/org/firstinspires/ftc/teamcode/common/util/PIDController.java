package org.firstinspires.ftc.teamcode.common.util;

import static org.firstinspires.ftc.teamcode.common.Globals.PID_THREAD_Hz;

import org.firstinspires.ftc.teamcode.common.frames.TaskLoopFrame;

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

    private double setpoint;
    private final DoubleSupplier pvProvider;
    private final DoubleSupplier timeProvider;
    private final DoubleConsumer outputConsumer;
    private final DoubleBinaryOperator feedforwardProvider;


    // 用于计算速度和加速度的状态变量
    private double previousPV = 0.0;
    private double previousVelocity = 0.0;
    private double currentVelocity = 0.0;
    private double currentAcceleration = 0.0;
    private boolean isFirstRun = true;

    private double lastOutput = 0;

    public static class DeltaNanoProvider implements DoubleSupplier {
        private double lastNano = System.nanoTime();
        @Override
        public double getAsDouble() {
            double delta = System.nanoTime() - lastNano;
            lastNano = System.nanoTime();
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
    }

    /**
     * PID 计算核心逻辑
     */
    public void compute() {

        double dt = timeProvider.getAsDouble();

        // 如果 dt 超过设定刷新时间的十倍，说明线程刚恢复或发生了严重卡顿
        if (dt > targetUpdateMs * 10) {
            dt = 0; // 丢弃这一帧的异常时间，防止积分爆炸
        }

        // 防止除以0或负时间导致的异常
        if (dt <= 0) {
            return;
        }

        double processVariable = pvProvider.getAsDouble();

        if (isFirstRun) {
            previousPV = processVariable;
            previousVelocity = 0.0;
            currentVelocity = 0.0;
            currentAcceleration = 0.0;
            isFirstRun = false;
        } else {
            currentVelocity = (processVariable - previousPV) / dt;
            currentAcceleration = (currentVelocity - previousVelocity) / dt;

            previousPV = processVariable;
            previousVelocity = currentVelocity;
        }
        

        // 1. 计算当前误差
        double error = setpoint - processVariable;

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

        if (lastOutput <= 1e-5) {
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
        compute();
    }

    /**
     * 默认在stop时会把电机功率设为0
     */
    @Override
    public void stop() {
        super.stop();
        outputConsumer.accept(0.0);
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
    public void start() {
        super.start();
        reset();
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
        return currentVelocity;
    }

    /**
     * 获取当前电机的加速度 (Process Variable 的二阶导数)
     * 单位：[PV单位] / 秒^2
     */
    public double getAcceleration() {
        return currentAcceleration;
    }

}
