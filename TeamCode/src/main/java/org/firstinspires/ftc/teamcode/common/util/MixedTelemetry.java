package org.firstinspires.ftc.teamcode.common.util;

public class MixedTelemetry {
    public org.firstinspires.ftc.robotcore.external.Telemetry officialTelemetry;

    public MixedTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry officialTelemetry) {
        this.officialTelemetry = officialTelemetry;
    }

    public void addLine(String value) {
        officialTelemetry.addLine(value);
    }


    public void addData(String key, Object data) {
        officialTelemetry.addData(key, data);
    }

    public void update() {
        officialTelemetry.update();
    }
}
