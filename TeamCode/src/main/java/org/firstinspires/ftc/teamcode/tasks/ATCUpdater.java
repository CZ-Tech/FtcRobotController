package org.firstinspires.ftc.teamcode.tasks;

import static org.firstinspires.ftc.teamcode.common.Globals.TARGET_ATC_UPDATE_FREQUENCY;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.drive.AdaptiveTractionController;

public class ATCUpdater {
    private boolean running = false;

    // 被读取的硬件
    private final AdaptiveTractionController ATC;
    private final int targetUpdateSpeed;

    private Thread updateThread;

    public static ATCUpdater getInstance() {
        return ATCUpdater.Instance.INSTANCE;
    }

    private static class Instance {
        public static final ATCUpdater INSTANCE = new ATCUpdater(TARGET_ATC_UPDATE_FREQUENCY, new AdaptiveTractionController(Robot.getInstance().odo));
    }

    /**
     * @param targetUpdateFrequency 循环速率，单位Hz
     */
    private ATCUpdater(int targetUpdateFrequency, AdaptiveTractionController ATC) {
        this.ATC = ATC;
        this.targetUpdateSpeed = 1000 / targetUpdateFrequency;
    }

    public synchronized void start() {
        running = true;
        updateThread = new Thread(() -> {
            long startTime = System.nanoTime() / 1_000_000L;
            while (running) {
                ATC.update();
                Robot.getInstance().telemetry.update();
                Robot.getInstance().gamepad1.update();
                Robot.getInstance().gamepad2.update();
                long sleepTime = targetUpdateSpeed - (System.nanoTime() / 1_000_000L - startTime);
                try {
                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime);
                    } else {
                        Robot.getInstance().telemetry
                                .addLine("last ATC took too much time!" +
                                        "it took {" + (System.nanoTime() / 1_000_000L - startTime) + "ms}");
                    }
                } catch (InterruptedException e) {
                    // 线程被中断，退出循环
                    Thread.currentThread().interrupt();
                    break;
                }
                startTime = System.nanoTime() / 1_000_000L;
            }
        });
        updateThread.setName("ATC-Updater");
//        updateThread.setDaemon(true); // 设置为守护线程，随主线程结束而结束
        updateThread.start();
    }

    public synchronized void stop() {
        running = false;
        if (updateThread != null) {
            updateThread.interrupt();
            try {
                updateThread.join(100); // 等待线程结束，最多100ms
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            updateThread = null;
        }
    }

    public boolean isRunning() {
        return running;
    }
}
