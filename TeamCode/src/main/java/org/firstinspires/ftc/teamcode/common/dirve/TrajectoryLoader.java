package org.firstinspires.ftc.teamcode.common.dirve;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Utility class to load trajectory keyframes from a JSON string and execute them using PinpointTrajectory.
 */
public class TrajectoryLoader {

    public static class Keyframe {
        public double x;
        public double y;
        public double dx;
        public double dy;
        public double heading;
        public double dHeading;
        public double duration;
        public double time;

        public Keyframe(JSONObject json) {
            this.x = json.optDouble("x", 0);
            this.y = json.optDouble("y", 0);
            this.dx = json.optDouble("dx", 0);
            this.dy = json.optDouble("dy", 0);
            this.heading = json.optDouble("heading", 0);
            this.dHeading = json.optDouble("dHeading", 0);
            this.duration = json.optDouble("duration", 0);
            this.time = json.optDouble("time", 0);
        }
    }

    /**
     * Parses the JSON keyframes and executes them on the provided PinpointTrajectory instance.
     * 
     * @param jsonString The JSON string containing an array of keyframes.
     * @param trajectory The PinpointTrajectory instance to execute on.
     */
    public static void executeJsonTrajectory(String jsonString, PinpointTrajectory trajectory) {
        try {
            JSONArray jsonArray = new JSONArray(jsonString);
            List<Keyframe> keyframes = new ArrayList<>();

            for (int i = 0; i < jsonArray.length(); i++) {
                keyframes.add(new Keyframe(jsonArray.getJSONObject(i)));
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
                if (i == 0) {
                    // 初始化起始点。注意：PinpointTrajectory 会将此点记为 startPoint
                    trajectory.startMove(kf.x, kf.y, kf.dx, kf.dy);
                    
                    // 同步初始目标朝向和追踪朝向，防止首段路径出现 360 度旋转 bug
                    trajectory.heading = kf.heading;
                    trajectory.preH = kf.heading; 
                } else {
                    // 按照时间线执行后续点
                    trajectory.addPoint(kf.time, kf.x, kf.y, kf.dx, kf.dy, kf.heading);
                }
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }
}
