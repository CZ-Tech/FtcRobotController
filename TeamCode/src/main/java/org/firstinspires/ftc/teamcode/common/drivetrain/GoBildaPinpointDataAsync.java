package org.firstinspires.ftc.teamcode.common.drivetrain;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.interfaces.IOdometer;

import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * 线程安全的里程计数据中专层。
 * <p>
 * 原版 GoBildaPinpointDriver 不做任何线程同步，
 * 此类通过读写锁+本地缓存解决多线程并发读取问题。
 * update() 应只由一个线程调用!!!
 * <p>
 */
public class GoBildaPinpointDataAsync implements IOdometer {
    private final GoBildaPinpointDriver odoUnAsync;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

    // 内部缓存，避免重复 I2C 通信
    private volatile GoBildaPinpointDriver.DeviceStatus deviceStatus;
    private volatile int loopTime       = 0;
    private volatile int xEncoderValue  = 0;
    private volatile int yEncoderValue  = 0;
    private volatile double xPosition    = 0;
    private volatile double yPosition    = 0;
    private volatile double hOrientation = 0;
    private volatile double xVelocity    = 0;
    private volatile double yVelocity    = 0;
    private volatile double hVelocity    = 0;

    // 速度 EMA 滤波
    private final double filterAlpha = 2.0 / (Globals.odoFilterWindowLength + 1);
    private double xVelFiltered = 0;
    private double yVelFiltered = 0;

    public GoBildaPinpointDataAsync(GoBildaPinpointDriver odoUnAsync) {
        this.odoUnAsync = odoUnAsync;
    }

    // ==========================================
    // IOdometer 实现
    // ==========================================

    /**
     * update() 应只由一个线程调用（主控制循环）。
     */
    @Override
    public void update() {
        odoUnAsync.update();
        updateData();
    }

    public void updateData() {
        lock.writeLock().lock();
        try {
            deviceStatus   = odoUnAsync.getDeviceStatus();
            loopTime       = odoUnAsync.getLoopTime();
            xEncoderValue  = odoUnAsync.getEncoderX();
            yEncoderValue  = odoUnAsync.getEncoderY();
            xPosition      = odoUnAsync.getPosX(DistanceUnit.MM);
            yPosition      = odoUnAsync.getPosY(DistanceUnit.MM);
            hOrientation   = odoUnAsync.getHeading(AngleUnit.RADIANS);

            double rawXVel = odoUnAsync.getVelX(DistanceUnit.MM);
            double rawYVel = odoUnAsync.getVelY(DistanceUnit.MM);
            xVelFiltered = filterAlpha * rawXVel + (1 - filterAlpha) * xVelFiltered;
            yVelFiltered = filterAlpha * rawYVel + (1 - filterAlpha) * yVelFiltered;
            xVelocity = xVelFiltered;
            yVelocity = yVelFiltered;

            hVelocity = odoUnAsync.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        } finally {
            lock.writeLock().unlock();
        }
    }


    @Override
    public Pose2D getPosition() {
        return getPosition(DistanceUnit.MM, AngleUnit.RADIANS);
    }

    public Pose2D getPosition(DistanceUnit distanceUnit, AngleUnit angleUnit) {
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

    @Override
    public double getHeadingRadians() {
        return getHeading(AngleUnit.RADIANS);
    }

    @Override
    public double getHeading(AngleUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromRadians(hOrientation);
        } finally {
            lock.readLock().unlock();
        }
    }

    public double getHeading(UnnormalizedAngleUnit unit) {
        lock.readLock().lock();
        try {
            return unit.fromRadians(hOrientation);
        } finally {
            lock.readLock().unlock();
        }
    }

    @Override
    public void resetPosAndIMU() {
        lock.writeLock().lock();
        try {
            odoUnAsync.resetPosAndIMU();
            Thread.sleep(500);
        } catch (InterruptedException ignored) {
        } finally {
            lock.writeLock().unlock();
        }
    }

    @Override
    public void resetHeading() {
        Pose2D cur = getPosition();
        setPosition(new Pose2D(
                DistanceUnit.MM,
                cur.getX(DistanceUnit.MM),
                cur.getY(DistanceUnit.MM),
                AngleUnit.RADIANS,
                0));
    }

    @Override
    public void setGlobalPose(Pose2D target) {
        setPosition(target);
    }

    // ==========================================
    // 硬件配置方法
    // ==========================================
    public void setOffsets(double xOffset, double yOffset) {
        setOffsets(xOffset, yOffset, DistanceUnit.MM);
    }

    public void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setOffsets(xOffset, yOffset, unit);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void recalibrateIMU() {
        lock.writeLock().lock();
        try {
            odoUnAsync.recalibrateIMU();
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xEncoder,
                                      GoBildaPinpointDriver.EncoderDirection yEncoder) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderDirections(xEncoder, yEncoder);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods pods) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderResolution(pods);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setEncoderResolution(double ticks_per_mm) {
        setEncoderResolution(ticks_per_mm, DistanceUnit.MM);
    }

    public void setEncoderResolution(double ticks_per_unit, DistanceUnit unit) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setEncoderResolution(ticks_per_unit, unit);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void setYawScalar(double yawOffset) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setYawScalar(yawOffset);
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * 强制设置 Pinpoint 当前位置姿态（直接写入硬件）。
     */
    public void setPosition(Pose2D currPos) {
        lock.writeLock().lock();
        try {
            odoUnAsync.setPosition(currPos);
            Thread.sleep(1000);
            Log.i("RobotLog setOdoPosition",
                    currPos.getX(DistanceUnit.INCH) + " " + currPos.getY(DistanceUnit.INCH));
            Log.i("setOdoPosition",
                    currPos.getX(DistanceUnit.INCH) + " " + currPos.getY(DistanceUnit.INCH));
            Log.i("odoPositionAfterSet",
                    odoUnAsync.getPosX(DistanceUnit.INCH) + " " +
                            odoUnAsync.getPosY(DistanceUnit.INCH) + " " +
                            odoUnAsync.getHeading(AngleUnit.DEGREES));
        } catch (InterruptedException ignored) {
        } finally {
            lock.writeLock().unlock();
        }
    }

    // ==========================================
    // 读取方法
    // ==========================================

    public int getDeviceID() {
        lock.readLock().lock();
        try { return odoUnAsync.getDeviceID(); }
        finally { lock.readLock().unlock(); }
    }

    public int getDeviceVersion() {
        lock.readLock().lock();
        try { return odoUnAsync.getDeviceVersion(); }
        finally { lock.readLock().unlock(); }
    }

    public float getYawScalar() {
        lock.readLock().lock();
        try { return odoUnAsync.getYawScalar(); }
        finally { lock.readLock().unlock(); }
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        lock.readLock().lock();
        try { return deviceStatus; }
        finally { lock.readLock().unlock(); }
    }

    public int getLoopTime() {
        lock.readLock().lock();
        try { return loopTime; }
        finally { lock.readLock().unlock(); }
    }

    public double getFrequency() {
        lock.readLock().lock();
        try { return loopTime != 0 ? 1000000.0 / loopTime : 0; }
        finally { lock.readLock().unlock(); }
    }

    public int getEncoderX() {
        lock.readLock().lock();
        try { return xEncoderValue; }
        finally { lock.readLock().unlock(); }
    }

    public int getEncoderY() {
        lock.readLock().lock();
        try { return yEncoderValue; }
        finally { lock.readLock().unlock(); }
    }

    public double getPosX() { return getPosX(DistanceUnit.MM); }

    public double getPosX(DistanceUnit unit) {
        lock.readLock().lock();
        try { return unit.fromMm(xPosition); }
        finally { lock.readLock().unlock(); }
    }

    public double getPosY() { return getPosY(DistanceUnit.MM); }

    public double getPosY(DistanceUnit unit) {
        lock.readLock().lock();
        try { return unit.fromMm(yPosition); }
        finally { lock.readLock().unlock(); }
    }

    public double getVelX() { return getVelX(DistanceUnit.MM); }

    public double getVelX(DistanceUnit unit) {
        lock.readLock().lock();
        try { return unit.fromMm(xVelocity); }
        finally { lock.readLock().unlock(); }
    }

    public double getVelY() { return getVelY(DistanceUnit.MM); }

    public double getVelY(DistanceUnit unit) {
        lock.readLock().lock();
        try { return unit.fromMm(yVelocity); }
        finally { lock.readLock().unlock(); }
    }

    public double getHeadingVelocity() { return getHeadingVelocity(AngleUnit.RADIANS); }

    public double getHeadingVelocity(AngleUnit unit) {
        lock.readLock().lock();
        try { return unit.fromRadians(hVelocity); }
        finally { lock.readLock().unlock(); }
    }

    public float getXOffset() { return getXOffset(DistanceUnit.MM); }

    public float getXOffset(DistanceUnit unit) {
        lock.readLock().lock();
        try { return odoUnAsync.getXOffset(unit); }
        finally { lock.readLock().unlock(); }
    }

    public float getYOffset() { return getYOffset(DistanceUnit.MM); }

    public float getYOffset(DistanceUnit unit) {
        lock.readLock().lock();
        try { return odoUnAsync.getYOffset(unit); }
        finally { lock.readLock().unlock(); }
    }

    public Pose2D getVelocity() { return getVelocity(DistanceUnit.MM, AngleUnit.RADIANS); }

    public Pose2D getVelocity(DistanceUnit distanceUnit, AngleUnit angleUnit) {
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
