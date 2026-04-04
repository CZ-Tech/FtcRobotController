package org.firstinspires.ftc.teamcode.common.frames;

import android.util.Log;


public abstract class TaskLoopFrame {
    private volatile boolean running = false;

    protected int targetUpdateMs;
    protected final String threadName;
    private Thread updateThread;

    /**
     * @param targetUpdateFrequency 循环速率，单位Hz
     * @param threadName 线程名，会关联logcat的日志输出消息
     */
    public TaskLoopFrame(int targetUpdateFrequency, String threadName) {
        this.threadName = threadName;
        this.targetUpdateMs = 1000 / targetUpdateFrequency;
    }

    public synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        updateThread = new Thread(() -> {
            long startTime = System.nanoTime() / 1_000_000L;
            while (running) {
                try {
                    update();
                } catch (Exception e) {
                    Log.d(threadName, e.getMessage()!=null ? e.getMessage() : threadName +" met an unexpected error");
                }
                long sleepTime = targetUpdateMs - (System.nanoTime() / 1_000_000L - startTime);
                try {
                    if (sleepTime > 0) {
                        Thread.sleep(sleepTime);
                    } else {
//                        Log.d(threadName, threadName +" Overloaded " +
//                                "last "+ threadName +" took too much time!" +
//                                "it took " + (System.nanoTime() / 1_000_000L - startTime) + " ms");
                    }
                } catch (InterruptedException e) {
                    // 线程被中断，退出循环
                    Thread.currentThread().interrupt();
                    running = false;
                    break;
                }
                startTime = System.nanoTime() / 1_000_000L;
            }
        });
        updateThread.setName(threadName);
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

    /**
     * 用于编写更新逻辑的方法，需要继承者实现
     * 调用start()后会尽可能以指定速率循环调用该方法
     * 但是没有滞后补偿机制
     */
    protected abstract void update();

}
