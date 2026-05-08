package org.firstinspires.ftc.teamcode.common.frames;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.frames.StoppableTask;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class TaskLoopFrame implements StoppableTask {
    // 防止任务在opMode结束后没有终止
    private static final List<StoppableTask> ALL_TASKS = Collections.synchronizedList(new ArrayList<>());

    private volatile boolean running = false;
    protected int targetUpdateMs;
    protected final String threadName;
    private Thread updateThread;
    private static int count = 0;

    public TaskLoopFrame(int targetUpdateFrequency, String threadName) {
        this.threadName = threadName;
        this.targetUpdateMs = 1000 / targetUpdateFrequency;
    }

    /**
     * 简便 API：使用 Lambda 快速创建任务
     */
    public static TaskLoopFrame create(int frequency, String name, Runnable action) {
        return new TaskLoopFrame(frequency, name) {
            @Override
            protected void update() {
                action.run();
            }
        };
    }

    /**
     * 简便 API：使用 Lambda 快速创建循环任务
     */
    public static TaskLoopFrame create(Runnable action) {
        return new TaskLoopFrame(20, "anonymous_task" + (count++)) {
            @Override
            protected void update() {
                action.run();
            }
        };
    }

    /**
     * 异步执行一次性任务，并在执行完成后自动停止和注销
     */
    public static void runOnce(Runnable action) {
        new TaskLoopFrame(100, "once_task" + (count++)) {
            @Override
            protected void update() {
                action.run();
                stop();
            }
        }.start();
    }

    public final synchronized void start() {
        if (running) return;
        onStart();
        running = true;
        ALL_TASKS.add(this);
        updateThread = new Thread(() -> {
            // 使用 nanoTime 保证精度
            long lastStartTime = System.nanoTime();
            long targetIntervalNs = targetUpdateMs * 1_000_000L;

            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    update();
                } catch (Exception e) {
                    Log.e(threadName, "Error in task: " + threadName, e);
                }

                long currentTime = System.nanoTime();
                long elapsedNs = currentTime - lastStartTime;
                long sleepNs = targetIntervalNs - elapsedNs;

                if (sleepNs > 0) {
                    try {
                        Thread.sleep(sleepNs / 1_000_000, (int) (sleepNs % 1_000_000));
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
                lastStartTime = System.nanoTime();
            }
            ALL_TASKS.remove(this);
        });
        updateThread.setName(threadName);
        updateThread.setPriority(Thread.NORM_PRIORITY); // 确保不会完全抢占硬件线程
        updateThread.start();
    }

    /**
     * 在线程每次启动时会调用该方法
     */
    protected void onStart() {

    }

    public final synchronized void stop() {
        onStop();
        running = false;
        if (updateThread != null) {
            updateThread.interrupt();
            updateThread = null;
        }
    }

    /**
     * 在线程每次停止时会调用该方法
     */
    protected void onStop() {

    }

    /**
     * 在 OpMode 结束时必须调用此方法
     * 会停止一切注册的后台任务
     */
    public static void stopAndClearAll() {
        // 复制一份防止遍历时修改的 ConcurrentModificationException
        StoppableTask[] tasks;
        synchronized (ALL_TASKS) {
            // 在锁内拿快照，保证拿到的是这一瞬间最完整的名单
            tasks = ALL_TASKS.toArray(new StoppableTask[0]);
        }
        for (StoppableTask t : tasks) {
            t.stop();
            ALL_TASKS.remove(t);
        }
    }

    public boolean isRunning() {
        return running;
    }

    protected abstract void update();
}