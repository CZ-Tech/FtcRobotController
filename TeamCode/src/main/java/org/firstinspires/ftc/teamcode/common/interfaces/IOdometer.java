package org.firstinspires.ftc.teamcode.common.interfaces;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 里程计接口 —— 融合后的场地位姿数据源。
 * <p>
 * 隐藏底层硬件细节（GoBilda Pinpoint、OTOS 等），
 * 上层组件只关心"位置在哪"，不关心"谁提供的"。
 * <p>
 * 参考实现：{@code MixedOdo}（偏移量融合架构）。
 */
public interface IOdometer {

    /** 刷新传感器数据，每个控制循环调用一次。 */
    void update();

    /** 当前融合后的场地位姿。 */
    Pose2D getPosition();

    /** 朝向角（弧度），便捷方法。 */
    double getHeadingRadians();

    /** 朝向角，指定单位。 */
    double getHeading(AngleUnit unit);

    /** 复位当前位置为零。 */
    void resetPosAndIMU();

    /** 仅复位朝向角为零，保持坐标不变。 */
    void resetHeading();

    /** 校准到已知场地位姿。 */
    void setGlobalPose(Pose2D target);
}
