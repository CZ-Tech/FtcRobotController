package org.firstinspires.ftc.teamcode.common.util;

import static org.firstinspires.ftc.teamcode.common.Globals.PID_THREAD_Hz;

import org.firstinspires.ftc.teamcode.common.frames.TaskLoopFrame;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * 增量式 PID 控制器 + 前馈
 */
public class IncrementalPIDController extends TaskLoopFrame {
    private double P;
    private double I;
    private double D;

    // 状态变量：增量式 PID 需要记录前两次的误差
    private double lastError = 0.0;
    private double lastLastError = 0.0;

    // PID 部分的累加器（相当于位置式 PID 的输出缓存）
    private double pidAccumulator = 0.0;

    // 积分限幅（在增量式中表现为对 PID 总贡献量的限幅）
    private double maxPIDContribution = 1.0;
    private double minPIDContribution = -1.0;

    private double setpoint;
    private final DoubleSupplier pvProvider;
    private final DoubleSupplier timeProvider;
    private final DoubleConsumer outputConsumer;
    private final DoubleBinaryOperator feedforwardProvider;

    private double lastTotalOutput = 0;

    /**
     * 时间提供者：将纳秒转换为秒，使 PID 参数处于合理的量级
     */
    public static class DeltaNanoProvider implements DoubleSupplier {
        private double lastNano = System.nanoTime();
        @Override
        public double getAsDouble() {
            long now = System.nanoTime();
            double delta = (now - lastNano) / 1e9; // 转换为秒 (seconds)
            lastNano = now;
            return delta;
        }
    }

    public void setPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public IncrementalPIDController(String PIDName, double P, double I, double D,
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
    }

    public IncrementalPIDController(String PIDName, double P, double I, double D,
                                    DoubleSupplier pvProvider,
                                    DoubleConsumer outputConsumer,
                                    DoubleBinaryOperator feedforwardProvider) {
        this(PIDName, P, I, D, pvProvider, new DeltaNanoProvider(), outputConsumer, feedforwardProvider);
    }

    public void setTarget(double target) {
        setpoint = target;
        this.start();
    }

    public double getTarget() {
        return setpoint;
    }

    /**
     * 在增量式 PID 中，限幅作用于 PID 累加器，防止过冲
     */
    public void setIntegralLimits(double min, double max) {
        this.minPIDContribution = min;
        this.maxPIDContribution = max;
    }

    public void reset() {
        this.pidAccumulator = 0.0;
        this.lastError = 0.0;
        this.lastLastError = 0.0;
        this.lastTotalOutput = 0.0;
    }

    /**
     * 核心计算逻辑：增量式 PID + 前馈叠加
     */
    public void compute() {
        double dt = timeProvider.getAsDouble();

        // 保护：防止 dt 过小或 provider 尚未初始化导致除以零
        if (dt <= 1e-6) {
            return;
        }

        double processVariable = pvProvider.getAsDouble();
        double error = setpoint - processVariable;

        // 1. 计算增量项 (Delta terms)
        // 增量式公式: ΔU = Kp*(e_k - e_k-1) + Ki*e_k*dt + Kd*(e_k - 2e_k-1 + e_k-2)/dt
        double deltaP = P * (error - lastError);
        double deltaI = I * error * dt;
        double deltaD = D * (error - 2 * lastError + lastLastError) / dt;

        // 2. 累加 PID 输出
        pidAccumulator += (deltaP + deltaI + deltaD);

        // 3. PID 部分限幅 (Anti-windup)
        pidAccumulator = Math.max(minPIDContribution, Math.min(maxPIDContribution, pidAccumulator));

        // 4. 计算前馈部分 (Feedforward)
        // 传入目标值和上一次的总输出
        double ffTerm = feedforwardProvider.applyAsDouble(setpoint, lastTotalOutput);

        // 5. 最终总输出 = PID累加量 + 前馈量
        // 这里限制在电机功率范围 [-1, 1]
        double finalOutput = Math.max(-1.0, Math.min(1.0, pidAccumulator + ffTerm));

        // 6. 更新状态，供下一帧使用
        lastLastError = lastError;
        lastError = error;
        lastTotalOutput = finalOutput;

        if (finalOutput <= 1e-5) {
            outputConsumer.accept(0.0);
            return;
        }

        // 7. 输出到电机
        outputConsumer.accept(finalOutput);
    }

    @Override
    public void update() {
        compute();
    }

    @Override
    public void onStop() {
        outputConsumer.accept(0.0);
    }

    public void stopPIDOnly() {
        super.stop();
    }

    public void breakMotor() {
        setpoint = 0;
    }

}
