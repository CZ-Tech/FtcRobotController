package org.firstinspires.ftc.teamcode.common.drive;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Utility class to load trajectory keyframes from a JSON string and execute them using PinpointTrajectory.
 */
public class TrajectoryLoader {

    private final PinpointTrajectory trajectory;
    private final Map<String, Runnable> markerTasks = new HashMap<>();

    public TrajectoryLoader(PinpointTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    /**
     * Registers a task to be executed when a keyframe with the specified marker is reached.
     * @param marker The marker name in the JSON.
     * @param task The task to execute.
     * @return This TrajectoryLoader instance for chaining.
     */
    public TrajectoryLoader addMarkerTask(String marker, Runnable task) {
        markerTasks.put(marker, task);
        return this;
    }

    public static class Keyframe {
        public double x;
        public double y;
        public double dx;
        public double dy;
        public double heading;
        public double dHeading;
        public double duration;
        public double time;
        public String marker;

        public Keyframe(JSONObject json) {
            this.x = json.optDouble("x", 0);
            this.y = json.optDouble("y", 0);
            this.dx = json.optDouble("dx", 0);
            this.dy = json.optDouble("dy", 0);
            this.heading = json.optDouble("heading", 0);
            this.dHeading = json.optDouble("dHeading", 0);
            this.duration = json.optDouble("duration", 0);
            this.time = json.optDouble("time", 0);
            this.marker = json.optString("marker", null);
        }
    }

    /**
     * Parses the JSON keyframes and executes them on the internal PinpointTrajectory instance.
     *
     * @param jsonString The JSON string containing an array of keyframes.
     */
    public void execute(String jsonString) {
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            List<Keyframe> keyframes = new ArrayList<>();

            double cumulativeTime = 0;
            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject obj = jsonArray.getJSONObject(i);
                Keyframe kf = new Keyframe(obj);
                if (obj.has("time")) {
                    cumulativeTime = kf.time;
                } else {
                    cumulativeTime += kf.duration;
                    kf.time = cumulativeTime;
                }
                keyframes.add(kf);
            }

            if (keyframes.isEmpty()) {
                return;
            }

            // Ensure they are sorted by time (Timeline mode)
            Collections.sort(keyframes, new Comparator<Keyframe>() {
                @Override
                public int compare(Keyframe k1, Keyframe k2) {
                    return Double.compare(k1.time, k2.time);
                }
            });

            trajectory.setMode(PinpointTrajectory.Mode.TIMELINE);

            for (int i = 0; i < keyframes.size(); i++) {
                Keyframe kf = keyframes.get(i);
                Runnable task = (kf.marker != null) ? markerTasks.get(kf.marker) : null;

                if (i == 0) {
                    // 同步物理位置到里程计系统，消除由于起始点不对齐导致的瞬时巨大误差
                    trajectory.setPose(new Pose2D(
                            org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                            kf.x, kf.y,
                            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                            kf.heading
                    ));

                    // 初始化起始点。注意：PinpointTrajectory 会将此点记为 startPoint
                    if (task != null) {
                        trajectory.startMove(kf.x, kf.y, kf.dx, kf.dy, task);
                    } else {
                        trajectory.startMove(kf.x, kf.y, kf.dx, kf.dy);
                    }

                    // 同步初始目标朝向和追踪朝向，防止首段路径出现 360 度旋转 bug
                    trajectory.heading = kf.heading;
                    trajectory.preH = kf.heading;
                } else {
                    // 按照时间线执行后续点
                    if (task != null) {
                        trajectory.addPoint(kf.time, kf.x, kf.y, kf.dx, kf.dy, kf.heading, task);
                    } else {
                        trajectory.addPoint(kf.time, kf.x, kf.y, kf.dx, kf.dy, kf.heading);
                    }
                }
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    /**
     * Static convenience method for backward compatibility.
     *
     * @param jsonString The JSON string containing an array of keyframes.
     * @param trajectory The PinpointTrajectory instance to execute on.
     */
    public static void executeJsonTrajectory(String jsonString, PinpointTrajectory trajectory) {
        new TrajectoryLoader(trajectory).execute(jsonString);
    }
}
