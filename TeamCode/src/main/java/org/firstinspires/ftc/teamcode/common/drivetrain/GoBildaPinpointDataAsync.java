package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.util.AvgOdoFilter;
import org.firstinspires.ftc.teamcode.common.util.OdoFilter;

import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * 这个类用于线程安全的获取编码器位置速度数据
 * 对于原版Driver中即使是有IO这般耗时操作都不愿意做线程安全
 * 我的评价是 FTC SDK F**K You !!
 */
public class GoBildaPinpointDataAsync {
    private final GoBildaPinpointDriver odoUnAsync;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final OdoFilter filter = new AvgOdoFilter(Globals.odoFilterWindowLength);

    // 内部变量，用于缓存从硬件批量读取的数据，避免重复的I2C通信。
    private GoBildaPinpointDriver.DeviceStatus deviceStatus;
    private volatile int loopTime       = 0;
    private volatile int xEncoderValue  = 0;
    private volatile int yEncoderValue  = 0;
    private volatile double xPosition    = 0;
    private volatile double yPosition    = 0;
    private volatile double hOrientation = 0;
    private volatile double xVelocity    = 0;
    private volatile double yVelocity    = 0;
    private volatile double hVelocity    = 0;

    private volatile double baseX = 0;    // MM
    private volatile double baseY = 0;    // MM
    private volatile double baseYaw = 0;  // RAD

    public GoBildaPinpointDataAsync(GoBildaPinpointDriver odoUnAsync) {
        this.odoUnAsync = odoUnAsync;
    }

    /**
     * update()应只由一个线程调用
     */
    public void update() {
        odoUnAsync.update();
        lock.writeLock().lock();
        try {
            deviceStatus   = odoUnAsync.getDeviceStatus();
            loopTime       = odoUnAsync.getLoopTime(); // Added
            xEncoderValue  = odoUnAsync.getEncoderX();
            yEncoderValue  = odoUnAsync.getEncoderY();
            xPosition      = odoUnAsync.getPosX(DistanceUnit.MM);
            yPosition      = odoUnAsync.getPosY(DistanceUnit.MM);
            hOrientation   = odoUnAsync.getHeading(AngleUnit.RADIANS);
            xVelocity      = filter.getFilteredSpeedX(odoUnAsync.getVelX(DistanceUnit.MM));
            yVelocity      = filter.getFilteredSpeedY(odoUnAsync.getVelY(DistanceUnit.MM));
            hVelocity      = odoUnAsync.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 向正方向旋转坐标系
     */
    private double[] transformCoordinate(double X, double Y, double heading) {
        double rotX = Y * Math.cos(heading) - X * Math.sin(heading);
        double rotY = Y * Math.sin(heading) + X * Math.cos(heading);
        return new double[] {rotX, rotY};
    }

    //    /**
//     * update()方法的一个重载版本，用于在需要更高读取速度时，仅更新部分数据。
//     * <p>
//     * 当前仅支持`ONLY_UPDATE_HEADING`，这会单独读取航向角数据，比批量读取所有数据要快。
//     * @param data 指定要更新的数据范围，目前仅支持 {@link GoBildaPinpointDriver.readData#ONLY_UPDATE_HEADING}
//     */
//    public void update(GoBildaPinpointDriver.readData data) {
//        lock.writeLock().lock();
//        try {
//            // This modifies the underlying driver's state, so it needs a write lock.
//            // After updating, we refresh our cached heading value.
//            odoUnAsync.update(data);
//            if (data == GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING) {
//                hOrientation = odoUnAsync.getHeading();
//            }
//            // Other cached values might become stale if only heading is updated.
//            // Consider if a full update() is needed periodically.
//        } finally {
//            lock.writeLock().unlock();
//        }
//    }

    /**
     * 设置里程计吊舱相对于机器人旋转中心的位置偏移量。
     * <p>
     * 坐标系定义：
     * - X轴（横向）：机器人中心的左侧为正，右侧为负。
     * - Y轴（纵向）：机器人中心的前方为正，后方为负。
     * @param xOffset X（前进）吊舱在横向上的偏移量（单位：毫米）。
     * @param yOffset Y（平移）吊舱在纵向上的偏移量（单位：毫米）。
     */
    public void setOffsets(double xOffset, double yOffset){
        setOffsets(xOffset, yOffset, DistanceUnit.MM);
    }

    /**
     * 设置里程计吊舱相对于机器人旋转中心的位置偏移量。
     * <p>
     * 坐标系定义：
     * - X轴（横向）：机器人中心的左侧为正，右侧为负。
     * - Y轴（纵向）：机器人中心的前方为正，后方为负。
     * @param xOffset X（前进）吊舱在横向上的偏移量。
     * @param yOffset Y（平移）吊舱在纵向上的偏移量。
     * @param unit 距离单位。
     */
    public void setOffsets(double xOffset, double yOffset, DistanceUnit unit){
        lock.writeLock().lock();
        try {
            odoUnAsync.setOffsets(xOffset, yOffset, unit);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 重新校准Pinpoint模块内置的IMU（陀螺仪）。
     * <p>
     * <strong>重要：执行此操作时，机器人必须保持完全静止。</strong>
     * <p>
     * 此过程会采集大量陀螺仪样本来计算零点漂移，大约需要0.25秒。
     */
    public void recalibrateIMU(){
        lock.writeLock().lock();
        try {
            odoUnAsync.recalibrateIMU();
            baseYaw = 0;
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 将当前位置重置为(0, 0, 0)并重新校准IMU。
     * <p>
     * <strong>重要：执行此操作时，机器人必须保持完全静止。</strong>
     * <p>
     * 这相当于同时执行了位置重置和IMU校准。
     */
    public void resetPosAndIMU(){
        lock.writeLock().lock();
        try {
            odoUnAsync.resetPosAndIMU();
            Thread.sleep(500);
            baseX = 0;
            baseY = 0;
            baseYaw = 0;
            // After resetting position, it's good practice to invalidate or reset cached position data.
            // For now, assume a full update will be called soon after.
        } catch (InterruptedException ignored) {

        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 设置两个编码器的计数方向。
     * <p>
     * 确保编码器方向设置正确，以获得准确的里程计数据。
     * @param xEncoder X（前进）编码器的方向。当机器人前进时，其计数值应增加。
     * @param yEncoder Y（平移）编码器的方向。当机器人向左平移时，其计数值应增加。
     */
    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xEncoder, GoBildaPinpointDriver.EncoderDirection yEncoder){
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderDirections(xEncoder, yEncoder);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 【便捷方法】如果使用goBILDA官方的里程计吊舱，可调用此方法快速设置编码器分辨率。
     * @param pods 吊舱类型，选择 {@link GoBildaPinpointDriver.GoBildaOdometryPods#goBILDA_SWINGARM_POD} 或 {@link GoBildaPinpointDriver.GoBildaOdometryPods#goBILDA_4_BAR_POD}
     */
    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods pods){
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderResolution(pods);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 自定义设置编码器的分辨率，单位为"每毫米的脉冲数"(ticks per mm)。
     * <p>
     * 计算公式：`ticks_per_mm = 编码器每转脉冲数(CPR) / 里程计轮周长(毫米)`。
     * @param ticks_per_mm 通常该值介于10到100之间。例如，goBILDA摆臂式吊舱的值约为13.26。
     */
    public void setEncoderResolution(double ticks_per_mm){
        setEncoderResolution(ticks_per_mm, DistanceUnit.MM);
    }

    /**
     * 自定义设置编码器的分辨率。
     * <p>
     * 计算公式：`ticks_per_unit = 编码器每转脉冲数(CPR) / 里程计轮周长(指定单位)`。
     * @param ticks_per_unit 每单位距离的脉冲数。
     * @param unit 距离单位。
     */
    public void setEncoderResolution(double ticks_per_unit, DistanceUnit unit){
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderResolution(ticks_per_unit, unit);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 设置偏航缩放系数（Yaw Scalar），用于微调航向角的测量精度。
     * <p>
     * <strong>通常无需调整此值</strong>，因为Pinpoint出厂时已经过校准。
     * <p>
     * 如果发现航向角有固定比例的累积误差，可以进行微调。例如，让机器人精确旋转10圈（3600度），
     * 如果测得值偏大或偏小，则用 `实际旋转角度 / 测量角度` 计算出新的缩放系数并设置。
     * 如果发现需要设置的系数大于1.05或小于0.95，可能表示设备存在问题。
     * @param yawOffset 航向角的缩放系数。
     */
    public void setYawScalar(double yawOffset){
        lock.writeLock().lock();
        try {
            odoUnAsync.setYawScalar(yawOffset);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 强制设置Pinpoint的当前位置姿态。
     * <p>
     * 此方法会完全覆盖Pinpoint内部的位置估算。主要用于以下两种场景：
     * <p>
     * <strong>1. 初始化场地坐标系：</strong> 在自动阶段开始时，根据机器人的起始位置，发送一个包含场地坐标的`Pose2D`，
     * 之后Pinpoint将基于此坐标系进行追踪。
     * <p>
     * <strong>2. 融合外部传感器数据：</strong> 当通过AprilTag、距离传感器等外部方式获得了更精确的绝对位置时，
     * 可调用此方法来修正Pinpoint的定位，Pinpoint会以此新位置为基准继续追踪。
     *
     * @param currPos 描述机器人新位置的`Pose2D`对象。
     */
    public void setPosition(Pose2D currPos){
        lock.writeLock().lock();
        try {
            odoUnAsync.setPosition(currPos);
            baseX = 0;
            baseY = 0;
            baseYaw = 0;
            Thread.sleep(1000);
            update();
            RobotLog.w("RobotLog setOdoPosition", currPos.getX(DistanceUnit.INCH) + " " + currPos.getY(DistanceUnit.INCH));
            Log.w("setOdoPosition", currPos.getX(DistanceUnit.INCH) + " " + currPos.getY(DistanceUnit.INCH));
            Log.w("odoPositionAfterSet", odoUnAsync.getPosX(DistanceUnit.INCH) + " " + odoUnAsync.getPosY(DistanceUnit.INCH) + " " + odoUnAsync.getHeading(AngleUnit.DEGREES));
//            baseX = odoUnAsync.getPosX(DistanceUnit.MM) - currPos.getX(DistanceUnit.MM);
//            baseY = odoUnAsync.getPosY(DistanceUnit.MM) - currPos.getY(DistanceUnit.MM);
//            baseYaw = odoUnAsync.getHeading(AngleUnit.RADIANS) - currPos.getHeading(AngleUnit.RADIANS);
        } catch (InterruptedException ignored) {

        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 获取设备ID。
     * @return 正常情况下应返回1。
     */
    public int getDeviceID(){
        lock.readLock().lock();
        try {
            return odoUnAsync.getDeviceID();
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取Pinpoint模块的固件版本号。
     * @return 固件版本号。
     */
    public int getDeviceVersion(){
        lock.readLock().lock();
        try {
            return odoUnAsync.getDeviceVersion();
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取当前设置的偏航缩放系数。
     * @return 偏航缩放系数。
     */
    public float getYawScalar(){
        lock.readLock().lock();
        try {
            return odoUnAsync.getYawScalar();
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取设备的当前状态，用于诊断故障。
     * @return 返回一个 {@link GoBildaPinpointDriver.DeviceStatus} 枚举值，可能的状态包括：<br>
     * - NOT_READY: 未就绪 (LED红色)<br>
     * - READY: 正常工作 (LED绿色)<br>
     * - CALIBRATING: 正在校准 (LED红色)<br>
     * - FAULT_NO_PODS_DETECTED: 未检测到任何吊舱 (LED紫色)<br>
     * - FAULT_X_POD_NOT_DETECTED: 未检测到X吊舱 (LED蓝色)<br>
     * - FAULT_Y_POD_NOT_DETECTED: 未检测到Y吊舱 (LED橙色)<br>
     */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        lock.readLock().lock();
        try {
            return deviceStatus; // This reads from cached value
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取Pinpoint内部控制循环的最近一次执行时间。
     * <p>
     * 如果该值通常小于500或大于1100，可能表示设备存在问题。
     * @return 循环时间，单位为微秒 (μs)。
     */
    public int getLoopTime() {
        lock.readLock().lock();
        try {
            return loopTime; // This reads from cached value
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取Pinpoint内部控制循环的频率。
     * <p>
     * 如果该值通常小于900Hz或大于2000Hz，可能表示设备存在问题。
     * @return 频率，单位为赫兹 (Hz)。
     */
    public double getFrequency(){
        lock.readLock().lock();
        try {
            if (loopTime != 0){
                return 1000000.0/loopTime;
            }
            else {
                return 0;
            }
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取X（前进）编码器的原始累计脉冲数（ticks）。
     */
    public int getEncoderX() {
        lock.readLock().lock();
        try {
            return xEncoderValue;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取Y（平移）编码器的原始累计脉冲数（ticks）。
     */
    public int getEncoderY() {
        lock.readLock().lock();
        try {
            return yEncoderValue;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的X（前进）方向位置。
     * @return X位置，单位：毫米 (mm)。
     */
    public double getPosX() {
        return getPosX(DistanceUnit.MM);
    }

    /**
     * 获取机器人估算的X（前进）方向位置。
     * @param unit 距离单位。
     * @return X位置，转换为指定单位。
     */
    public double getPosX(DistanceUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromMm(xPosition);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的Y（平移）方向位置。
     * @return Y位置，单位：毫米 (mm)。
     */
    public double getPosY() {
        return getPosY(DistanceUnit.MM);
    }

    /**
     * 获取机器人估算的Y（平移）方向位置。
     * @param unit 距离单位。
     * @return Y位置，转换为指定单位。
     */
    public double getPosY(DistanceUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromMm(yPosition);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的航向角。
     * @return 航向角，单位：弧度 (radians)。
     */
    public double getHeading() {
        return getHeading(AngleUnit.RADIANS);
    }

    /**
     * 获取机器人估算的航向角。
     * @param unit 角度单位。
     * @return 航向角，转换为指定单位。
     */
    public double getHeading(AngleUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromRadians(hOrientation);
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        lock.readLock().lock();
        try {
            return unnormalizedAngleUnit.fromRadians(hOrientation);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的X（前进）方向速度。
     * @return X速度，单位：毫米/秒 (mm/s)。
     */
    public double getVelX() {
        return getVelX(DistanceUnit.MM);
    }

    /**
     * 获取机器人估算的X（前进）方向速度。
     * @param unit 距离单位。
     * @return X速度，转换为指定单位/秒。
     */
    public double getVelX(DistanceUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromMm(xVelocity);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的Y（平移）方向速度。
     * @return Y速度，单位：毫米/秒 (mm/s)。
     */
    public double getVelY() {
        return getVelY(DistanceUnit.MM);
    }

    /**
     * 获取机器人估算的Y（平移）方向速度。
     * @param unit 距离单位。
     * @return Y速度，转换为指定单位/秒。
     */
    public double getVelY(DistanceUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromMm(yVelocity);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取机器人估算的航向角速度。
     * @return 角速度，单位：弧度/秒 (radians/s)。
     */
    public double getHeadingVelocity() {
        return getHeadingVelocity(AngleUnit.RADIANS);
    }

    /**
     * 获取机器人估算的航向角速度。
     * @param unit 角度单位。
     * @return 角速度，转换为指定单位/秒。
     */
    public double getHeadingVelocity(AngleUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromRadians(hVelocity);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取用户设置的X（前进）吊舱的偏移量 (毫米)。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @return X偏移量。
     */
    public float getXOffset(){
        return getXOffset(DistanceUnit.MM);
    }

    /**
     * 获取用户设置的X（前进）吊舱的偏移量。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @param unit 距离单位。
     * @return X偏移量，转换为指定单位。
     */
    public float getXOffset(DistanceUnit unit){
        lock.readLock().lock();
        try {
            return odoUnAsync.getXOffset(unit);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取用户设置的Y（平移）吊舱的偏移量 (毫米)。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @return Y偏移量。
     */
    public float getYOffset(){
        return getYOffset(DistanceUnit.MM);
    }

    /**
     * 获取用户设置的Y（平移）吊舱的偏移量。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @param unit 距离单位。
     * @return Y偏移量，转换为指定单位。
     */
    public float getYOffset(DistanceUnit unit){
        lock.readLock().lock();
        try {
            return odoUnAsync.getYOffset(unit);
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取一个包含机器人完整位置信息的`Pose2D`对象。
     * @return 包含X、Y位置和航向角的`Pose2D`对象（单位：毫米和弧度）。
     */
    public Pose2D getPosition(){
        return getPosition(DistanceUnit.MM, AngleUnit.RADIANS);
    }

    /**
     * 获取一个包含机器人完整位置信息的`Pose2D`对象。
     * @param distanceUnit 距离单位。
     * @param angleUnit 角度单位。
     * @return 包含X、Y位置和航向角的`Pose2D`对象，使用指定单位。
     */
    public Pose2D getPosition(DistanceUnit distanceUnit, AngleUnit angleUnit){
        lock.readLock().lock();
        try {
            return new Pose2D(
                    distanceUnit,
                    distanceUnit.fromMm(xPosition),
                    distanceUnit.fromMm(yPosition),
                    angleUnit,
                    angleUnit.fromRadians(hOrientation));
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * 获取一个包含机器人完整速度信息的`Pose2D`对象。
     * @return 包含X、Y线速度和航向角速度的`Pose2D`对象。速度单位为 毫米/秒 和 弧度/秒。
     */
    public Pose2D getVelocity(){
        return getVelocity(DistanceUnit.MM, AngleUnit.RADIANS);
    }

    /**
     * 获取一个包含机器人完整速度信息的`Pose2D`对象。
     * @param distanceUnit 距离单位。
     * @param angleUnit 角度单位。
     * @return 包含X、Y线速度和航向角速度的`Pose2D`对象，使用指定单位。
     */
    public Pose2D getVelocity(DistanceUnit distanceUnit, AngleUnit angleUnit){
        lock.readLock().lock();
        try {
            return new Pose2D(
                    distanceUnit,
                    distanceUnit.fromMm(xVelocity),
                    distanceUnit.fromMm(yVelocity),
                    angleUnit,
                    angleUnit.fromRadians(hVelocity));
        } finally {
            lock.readLock().unlock();
        }
    }

}