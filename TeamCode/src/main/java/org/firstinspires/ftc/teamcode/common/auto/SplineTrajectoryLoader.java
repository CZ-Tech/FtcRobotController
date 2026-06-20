package org.firstinspires.ftc.teamcode.common.auto;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.frames.HttpJsonService;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

/**
 * SplineTracker 专用的轨迹加载器。
 *
 * 与 {@link TrajectoryLoader} 的区别：
 * - 不解析 time/duration（SplineTracker 是位置驱动，自动走到终点）
 * - 支持两种关键帧：路径点（有 x,y）和等待点（有 wait）
 * - marker 机制与原有相同
 */
public class SplineTrajectoryLoader {

    private final SplineTracker tracker;
    private final Map<String, Runnable> markerTasks = new HashMap<>();

    public SplineTrajectoryLoader(SplineTracker tracker) {
        this.tracker = tracker;
    }

    /**
     * 注册标记任务。
     * @param marker JSON 中的 marker 名称
     * @param task   到达该标记时执行的任务
     * @return this（链式调用）
     */
    public SplineTrajectoryLoader addMarkerTask(String marker, Runnable task) {
        markerTasks.put(marker, task);
        return this;
    }

    /**
     * 解析 JSON 并驱动 SplineTracker。
     *
     * JSON 格式示例：
     * <pre>
     * [
     *     {"x": 0,  "y": 0,  "dx": 0, "dy": 0, "heading": 0},
     *     {"x": 24, "y": 0,  "dx": 0, "dy": 0, "heading": 0, "marker": "shoot"},
     *     {"wait": 1.5},
     *     {"x": 48, "y": 24, "dx": 0, "dy": 0, "heading": 90},
     *     {"wait": 2.0, "marker": "done"}
     * ]
     * </pre>
     *
     * @param jsonString 轨迹 JSON 字符串
     */
    public void execute(String jsonString) {
        HttpJsonService.scanObjectTree(tracker.getRobot());
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            if (jsonArray.length() == 0) return;

            boolean isFirstWaypoint = true;

            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject obj = jsonArray.getJSONObject(i);

                if (obj.has("wait")) {
                    // --- 等待点 ---
                    double waitSeconds = obj.getDouble("wait");
                    String marker = obj.optString("marker", null);
                    Runnable task = (marker != null) ? markerTasks.get(marker) : null;

                    if (task != null) {
                        Log.i("SplineAuto", marker);
                        tracker.addTime(waitSeconds, task);
                    } else {
                        tracker.addTime(waitSeconds);
                    }

                } else {
                    // --- 路径点 ---
                    double x = obj.getDouble("x");
                    double y = obj.getDouble("y");
                    double dx = obj.optDouble("dx", 0);
                    double dy = obj.optDouble("dy", 0);
                    double heading = obj.optDouble("heading", 0);
                    String marker = obj.optString("marker", null);
                    Runnable task = (marker != null) ? markerTasks.get(marker) : null;

                    if (isFirstWaypoint) {
                        // 第一个路径点：设置初始位置
                        tracker.setPose(new Pose2D(
                                DistanceUnit.INCH, x, y,
                                AngleUnit.DEGREES, heading
                        ));

                        if (task != null) {
                            Log.i("SplineAuto", marker);
                            tracker.startMove(x, y, dx, dy, task);
                        } else {
                            tracker.startMove(x, y, dx, dy);
                        }

                        // 同步朝向，覆盖 startMove 从里程计读取的朝向
                        tracker.heading = heading;
                        tracker.preH = heading;

                        isFirstWaypoint = false;
                    } else {
                        if (task != null) {
                            Log.i("SplineAuto", marker);
                            tracker.addPoint(x, y, dx, dy, heading, task);
                        } else {
                            tracker.addPoint(x, y, dx, dy, heading);
                        }
                    }
                }
            }

        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    /**
     * 静态便捷方法。
     */
    public static void executeJsonTrajectory(String jsonString, SplineTracker tracker) {
        new SplineTrajectoryLoader(tracker).execute(jsonString);
    }
}
