//package org.firstinspires.ftc.teamcode.common.frames;
//
//import android.content.Context;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
//import android.util.Log;
//
//public class MyBackgroundService {
//
//    private static boolean isStarted = false;
//
//    /**
//     * 当 RC App 启动并扫描 OpMode 时，此方法会被自动调用
//     * 它不依赖于任何具体的 OpMode 运行
//     */
//    @OpModeRegistrar
//    public static void initService(Context context, OpModeManager manager) {
//        if (!isStarted) {
//            // 启动一个独立的后台线程
//            Thread serviceThread = new Thread(() -> {
//                while (!Thread.currentThread().isInterrupted()) {
//                    try {
//                        // 这里写你的后台逻辑，比如心跳检测或数据上报
//                        Log.d("MyService", "后台服务正在运行中... 当前时间: " + System.currentTimeMillis());
//
//                        Thread.sleep(2000); // 每2秒运行一次
//                    } catch (InterruptedException e) {
//                        break;
//                    }
//                }
//            });
//
//            serviceThread.setDaemon(true); // 设置为守护线程，随 App 退出而结束
//            serviceThread.start();
//
//            isStarted = true;
//            Log.d("MyService", "--- 后台服务已成功启动 ---");
//        }
//    }
//}