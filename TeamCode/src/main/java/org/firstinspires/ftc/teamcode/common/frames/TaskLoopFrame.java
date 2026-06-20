package org.firstinspires.ftc.teamcode.common.frames;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.interfaces.Command;
import org.firstinspires.ftc.teamcode.common.interfaces.StoppableTask;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public abstract class TaskLoopFrame implements Command {
    // 防止任务在opMode结束后没有终止
    private static final List<StoppableTask> ALL_TASKS = Collections.synchronizedList(new ArrayList<>());
    private static final AtomicInteger pendingNonLoopingTasks = new AtomicInteger(0);

    private volatile boolean running = false;
    private volatile boolean looping = true;
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
        TaskLoopFrame task = new TaskLoopFrame(100, "once_task" + (count++)) {
            @Override
            protected void update() {
                action.run();
                stop();
            }
        };
        task.looping = false;
        pendingNonLoopingTasks.incrementAndGet();
        task.start();
    }

    @Override
    public void execute() {
        update();
    }

    public final synchronized void start() {
        if (running) return;
        init();     // WPILib 生命周期：初始化
        onStart();
        running = true;
        ALL_TASKS.add(this);
        updateThread = new Thread(() -> {
            // 使用 nanoTime 保证精度
            long lastStartTime = System.nanoTime();
            long targetIntervalNs = targetUpdateMs * 1_000_000L;

            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    execute();      // WPILib 生命周期：周期性执行
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
            if (!looping) {
                pendingNonLoopingTasks.decrementAndGet();
            }
        });
        updateThread.setName(threadName);
        updateThread.setPriority(Thread.NORM_PRIORITY); // 确保不会完全抢占硬件线程
        updateThread.start();
    }

    /**
     * 在线程每次启动时会调用该方法
     * <p>
     * WARN: 当线程已经处于启动状态时再调用start()不会触发此方法
     */
    protected void onStart() {

    }

    @Override
    public final synchronized void stop() {
        onStop();
        end(true);  // WPILib 生命周期：以中断方式结束
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
        pendingNonLoopingTasks.set(0);
    }

    /**
     * 等待所有非循环任务（如 runOnce 创建的一次性任务）执行完毕
     * 会阻塞当前线程直到所有非循环任务完成
     */
    public static void joinAllTask() {
        joinAllTask(0);
    }

    /**
     * 等待所有非循环任务执行完毕，超时后强制停止剩余任务
     *
     * @param timeoutMs 超时时间（毫秒），<= 0 表示无限等待
     */
    public static void joinAllTask(long timeoutMs) {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;

        while (pendingNonLoopingTasks.get() > 0) {
            if (timeoutMs > 0 && System.nanoTime() >= deadline) {
                stopAllNonLooping();
                break;
            }
            Robot.sleep(10);
        }
    }

    /**
     * 强制停止所有非循环任务
     */
    private static void stopAllNonLooping() {
        StoppableTask[] tasks;
        synchronized (ALL_TASKS) {
            tasks = ALL_TASKS.toArray(new StoppableTask[0]);
        }
        for (StoppableTask t : tasks) {
            if (t instanceof TaskLoopFrame && !((TaskLoopFrame) t).looping) {
                t.stop();
            }
        }
    }

    public boolean isRunning() {
        return running;
    }

    protected abstract void update();
}