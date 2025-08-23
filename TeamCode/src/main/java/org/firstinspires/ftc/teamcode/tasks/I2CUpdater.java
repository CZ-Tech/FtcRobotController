package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDataAsync;
import org.firstinspires.ftc.teamcode.common.hardware.GoBildaPinpointDriver;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/*
 * 该类用于管理i2c数据读取
 */
public class I2CUpdater {
    private I2CUpdater INSTANCE;

    private boolean running = false;

    // 被读取的硬件
    private final GoBildaPinpointDataAsync odo;
    private final int targetUpdateSpeed;

    public I2CUpdater getInstance() {
        return INSTANCE;
    }

    /**
     * 该方法将立即开始循环线程并先空转
     * @param targetUpdateFrequency 循环速率，单位Hz
     */
    private I2CUpdater(int targetUpdateFrequency, GoBildaPinpointDataAsync odo) {
        this.odo = odo;
        this.targetUpdateSpeed = 1000 / targetUpdateFrequency;
    }

    public void start() {
        running = true;
        new Thread(() -> {
            while (true) {
                long startTime = System.nanoTime();
                odo.update();
                long sleepTime = targetUpdateSpeed - System.nanoTime() - startTime;
                try {
                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime / 1_000_000L);
                    } else {
                        Robot.getInstance().telemetry
                                .addLine("last I2C took too much time!" +
                                "it took {" + (System.nanoTime() - startTime) / 1_000_000L + "ms}");
                    }
                } catch (InterruptedException e) {
                    Robot.getInstance().telemetry
                            .addLine("unexpected threading err occurred. message:" +
                                    e.getMessage());
                }
            }
        }).start();
    }

}
