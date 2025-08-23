/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.common.hardware;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
/**
 * goBILDA® Pinpoint 硬件模块的驱动程序。
 * <p>
 * 该类负责与Pinpoint模块进行底层I2C通信，将硬件返回的原始数据解析为Java代码中易于使用的格式。
 * 它封装了复杂的通信协议，并提供了高级API来获取机器人的位置、速度、设备状态以及进行相关配置。
 * Pinpoint模块本身完成了所有传感器融合和运动学计算，该驱动程序主要充当代码与硬件之间的桥梁。
 */
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    // 内部变量，用于缓存从硬件批量读取的数据，避免重复的I2C通信。
    private int deviceStatus   = 0;
    private int loopTime       = 0;
    private int xEncoderValue  = 0;
    private int yEncoderValue  = 0;
    private float xPosition    = 0;
    private float yPosition    = 0;
    private float hOrientation = 0;
    private float xVelocity    = 0;
    private float yVelocity    = 0;
    private float hVelocity    = 0;

    // goBILDA官方里程计吊舱的每毫米脉冲数（ticks-per-mm）常量
    private static final float goBILDA_SWINGARM_POD = 13.26291192f; // 适用于goBILDA摆臂式吊舱
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; // 适用于goBILDA四连杆式吊舱

    // Pinpoint模块的默认I2C设备地址
    public static final byte DEFAULT_ADDRESS = 0x31;

    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        // 设置I2C总线速度为400KHz快速模式以优化通信
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }


    // Pinpoint模块的I2C寄存器地址映射
    private enum Register {
        DEVICE_ID       (1),  // 设备ID
        DEVICE_VERSION  (2),  // 固件版本
        DEVICE_STATUS   (3),  // 设备状态
        DEVICE_CONTROL  (4),  // 设备控制
        LOOP_TIME       (5),  // 内部循环时间
        X_ENCODER_VALUE (6),  // X编码器原始值
        Y_ENCODER_VALUE (7),  // Y编码器原始值
        X_POSITION      (8),  // X位置
        Y_POSITION      (9),  // Y位置
        H_ORIENTATION   (10), // 航向角
        X_VELOCITY      (11), // X速度
        Y_VELOCITY      (12), // Y速度
        H_VELOCITY      (13), // 航向角速度
        MM_PER_TICK     (14), // 每毫米脉冲数
        X_POD_OFFSET    (15), // X吊舱偏移量
        Y_POD_OFFSET    (16), // Y吊舱偏移量
        YAW_SCALAR      (17), // 偏航缩放系数
        BULK_READ       (18); // 批量读取起始地址

        private final int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    // 设备状态枚举，用于描述设备当前的工作或故障状态
    public enum DeviceStatus{
        NOT_READY                (0),      // 未就绪
        READY                    (1),      // 已就绪
        CALIBRATING              (1 << 1), // 正在校准
        FAULT_X_POD_NOT_DETECTED (1 << 2), // 故障：未检测到X吊舱
        FAULT_Y_POD_NOT_DETECTED (1 << 3), // 故障：未检测到Y吊舱
        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3), // 故障：未检测到任何吊舱
        FAULT_IMU_RUNAWAY        (1 << 4); // 故障：IMU数据异常

        private final int status;

        DeviceStatus(int status){
            this.status = status;
        }
    }

    // 编码器方向枚举
    public enum EncoderDirection{
        FORWARD,  // 正向
        REVERSED; // 反向
    }

    // goBILDA官方里程计吊舱类型枚举
    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD, // 摆臂式吊舱
        goBILDA_4_BAR_POD;    // 四连杆式吊舱
    }

    // 数据读取范围枚举，用于优化update调用
    public enum readData {
        ONLY_UPDATE_HEADING, // 仅更新航向角数据
    }


    /**
     * [内部辅助方法] 向指定的I2C寄存器写入一个整数。
     * @param reg 目标寄存器
     * @param i   要写入的整数
     */
    private void writeInt(final Register reg, int i){
        deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i,ByteOrder.LITTLE_ENDIAN));
    }

    /**
     * [内部辅助方法] 从指定的I2C寄存器读取一个整数。
     * @param reg 目标寄存器
     * @return 从寄存器读取到的整数值
     */
    private int readInt(Register reg){
        return byteArrayToInt(deviceClient.read(reg.bVal,4), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * [内部辅助方法] 将字节数组转换为浮点数。
     * @param byteArray 待转换的字节数组
     * @param byteOrder 字节序
     * @return 转换后的浮点数值
     */
    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder){
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    /**
     * [内部辅助方法] 从指定的I2C寄存器读取一个浮点数。
     * @param reg 目标寄存器
     * @return 从寄存器读取到的浮点数值
     */
    private float readFloat(Register reg){
        return byteArrayToFloat(deviceClient.read(reg.bVal,4),ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * [内部辅助方法] 将浮点数转换为字节数组。
     * @param value 待转换的浮点数
     * @return 转换后的字节数组
     */
    private byte [] floatToByteArray (float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    /**
     * [内部辅助方法] 向指定的I2C寄存器写入一个字节数组。
     * @param reg   目标寄存器
     * @param bytes 要写入的字节数组
     */
    private void writeByteArray (Register reg, byte[] bytes){
        deviceClient.write(reg.bVal,bytes);
    }

    /**
     * [内部辅助方法] 向指定的I2C寄存器写入一个浮点数。
     * @param reg 目标寄存器
     * @param f   要写入的浮点数
     */
    private void writeFloat (Register reg, float f){
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        deviceClient.write(reg.bVal,bytes);
    }

    /**
     * [内部辅助方法] 将从设备读取的原始状态码（整数）转换为可读的DeviceStatus枚举。
     * @param s 原始状态码
     * @return 对应的DeviceStatus枚举值
     */
    private DeviceStatus lookupStatus (int s){
        if ((s & DeviceStatus.CALIBRATING.status) != 0){
            return DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if(!xPodDetected  && !yPodDetected){
            return DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected){
            return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected){
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0){
            return DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & DeviceStatus.READY.status) != 0){
            return DeviceStatus.READY;
        }
        else {
            return DeviceStatus.NOT_READY;
        }
    }

    /**
     * 在每个控制循环中调用此方法来刷新所有运动数据。
     * <p>
     * 该方法通过一次I2C“批量读取”操作，从Pinpoint模块获取所有最新的数据（包括位置、速度、编码器值、设备状态等），
     * 并更新到该类的内部变量中。
     */
    public void update(){
        // 从BULK_READ寄存器开始，一次性读取40个字节
        byte[] bArr   = deviceClient.read(Register.BULK_READ.bVal, 40);
        // 按顺序解析字节数组，填充内部变量
        deviceStatus  = byteArrayToInt(Arrays.copyOfRange  (bArr, 0, 4),  ByteOrder.LITTLE_ENDIAN);
        loopTime      = byteArrayToInt(Arrays.copyOfRange  (bArr, 4, 8),  ByteOrder.LITTLE_ENDIAN);
        xEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 8, 12), ByteOrder.LITTLE_ENDIAN);
        yEncoderValue = byteArrayToInt(Arrays.copyOfRange  (bArr, 12,16), ByteOrder.LITTLE_ENDIAN);
        xPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 16,20), ByteOrder.LITTLE_ENDIAN);
        yPosition     = byteArrayToFloat(Arrays.copyOfRange(bArr, 20,24), ByteOrder.LITTLE_ENDIAN);
        hOrientation  = byteArrayToFloat(Arrays.copyOfRange(bArr, 24,28), ByteOrder.LITTLE_ENDIAN);
        xVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 28,32), ByteOrder.LITTLE_ENDIAN);
        yVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 32,36), ByteOrder.LITTLE_ENDIAN);
        hVelocity     = byteArrayToFloat(Arrays.copyOfRange(bArr, 36,40), ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * update()方法的一个重载版本，用于在需要更高读取速度时，仅更新部分数据。
     * <p>
     * 当前仅支持`ONLY_UPDATE_HEADING`，这会单独读取航向角数据，比批量读取所有数据要快。
     * @param data 指定要更新的数据范围，目前仅支持 {@link readData#ONLY_UPDATE_HEADING}
     */
    public void update(readData data) {
        if (data == readData.ONLY_UPDATE_HEADING) {
            hOrientation = byteArrayToFloat(deviceClient.read(Register.H_ORIENTATION.bVal, 4), ByteOrder.LITTLE_ENDIAN);
        }
    }

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
        writeFloat(Register.X_POD_OFFSET, (float) xOffset);
        writeFloat(Register.Y_POD_OFFSET, (float) yOffset);
    }

    /**
     * 重新校准Pinpoint模块内置的IMU（陀螺仪）。
     * <p>
     * <strong>重要：执行此操作时，机器人必须保持完全静止。</strong>
     * <p>
     * 此过程会采集大量陀螺仪样本来计算零点漂移，大约需要0.25秒。
     */
    public void recalibrateIMU(){writeInt(Register.DEVICE_CONTROL,1<<0);}

    /**
     * 将当前位置重置为(0, 0, 0)并重新校准IMU。
     * <p>
     * <strong>重要：执行此操作时，机器人必须保持完全静止。</strong>
     * <p>
     * 这相当于同时执行了位置重置和IMU校准。
     */
    public void resetPosAndIMU(){writeInt(Register.DEVICE_CONTROL,1<<1);}

    /**
     * 设置两个编码器的计数方向。
     * <p>
     * 确保编码器方向设置正确，以获得准确的里程计数据。
     * @param xEncoder X（前进）编码器的方向。当机器人前进时，其计数值应增加。
     * @param yEncoder Y（平移）编码器的方向。当机器人向左平移时，其计数值应增加。
     */
    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder){
        if (xEncoder == EncoderDirection.FORWARD){
            writeInt(Register.DEVICE_CONTROL,1<<5);
        }
        if (xEncoder == EncoderDirection.REVERSED) {
            writeInt(Register.DEVICE_CONTROL,1<<4);
        }

        if (yEncoder == EncoderDirection.FORWARD){
            writeInt(Register.DEVICE_CONTROL,1<<3);
        }
        if (yEncoder == EncoderDirection.REVERSED){
            writeInt(Register.DEVICE_CONTROL,1<<2);
        }
    }

    /**
     * 【便捷方法】如果使用goBILDA官方的里程计吊舱，可调用此方法快速设置编码器分辨率。
     * @param pods 吊舱类型，选择 {@link GoBildaOdometryPods#goBILDA_SWINGARM_POD} 或 {@link GoBildaOdometryPods#goBILDA_4_BAR_POD}
     */
    public void setEncoderResolution(GoBildaOdometryPods pods){
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_SWINGARM_POD, ByteOrder.LITTLE_ENDIAN)));
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD){
            writeByteArray(Register.MM_PER_TICK,(floatToByteArray(goBILDA_4_BAR_POD, ByteOrder.LITTLE_ENDIAN)));
        }
    }

    /**
     * 自定义设置编码器的分辨率，单位为“每毫米的脉冲数”(ticks per mm)。
     * <p>
     * 计算公式：`ticks_per_mm = 编码器每转脉冲数(CPR) / 里程计轮周长(毫米)`。
     * @param ticks_per_mm 通常该值介于10到100之间。例如，goBILDA摆臂式吊舱的值约为13.26。
     */
    public void setEncoderResolution(double ticks_per_mm){
        writeByteArray(Register.MM_PER_TICK,(floatToByteArray((float) ticks_per_mm,ByteOrder.LITTLE_ENDIAN)));
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
        writeByteArray(Register.YAW_SCALAR,(floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN)));
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
     * @param pos 描述机器人新位置的`Pose2D`对象。
     * @return 返回设置的`Pose2D`对象。
     */
    public Pose2D setPosition(Pose2D pos){
        writeByteArray(Register.X_POSITION,(floatToByteArray((float) pos.getX(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.Y_POSITION,(floatToByteArray((float) pos.getY(DistanceUnit.MM),ByteOrder.LITTLE_ENDIAN)));
        writeByteArray(Register.H_ORIENTATION,(floatToByteArray((float) pos.getHeading(AngleUnit.RADIANS),ByteOrder.LITTLE_ENDIAN)));
        return pos;
    }

    /**
     * 获取设备ID。
     * @return 正常情况下应返回1。
     */
    public int getDeviceID(){return readInt(Register.DEVICE_ID);}

    /**
     * 获取Pinpoint模块的固件版本号。
     * @return 固件版本号。
     */
    public int getDeviceVersion(){return readInt(Register.DEVICE_VERSION); }

    /**
     * 获取当前设置的偏航缩放系数。
     * @return 偏航缩放系数。
     */
    public float getYawScalar(){return readFloat(Register.YAW_SCALAR); }

    /**
     * 获取设备的当前状态，用于诊断故障。
     * @return 返回一个 {@link DeviceStatus} 枚举值，可能的状态包括：<br>
     * - NOT_READY: 未就绪 (LED红色)<br>
     * - READY: 正常工作 (LED绿色)<br>
     * - CALIBRATING: 正在校准 (LED红色)<br>
     * - FAULT_NO_PODS_DETECTED: 未检测到任何吊舱 (LED紫色)<br>
     * - FAULT_X_POD_NOT_DETECTED: 未检测到X吊舱 (LED蓝色)<br>
     * - FAULT_Y_POD_NOT_DETECTED: 未检测到Y吊舱 (LED橙色)<br>
     */
    public DeviceStatus getDeviceStatus(){return lookupStatus(deviceStatus); }

    /**
     * 获取Pinpoint内部控制循环的最近一次执行时间。
     * <p>
     * 如果该值通常小于500或大于1100，可能表示设备存在问题。
     * @return 循环时间，单位为微秒 (μs)。
     */
    public int getLoopTime(){return loopTime; }

    /**
     * 获取Pinpoint内部控制循环的频率。
     * <p>
     * 如果该值通常小于900Hz或大于2000Hz，可能表示设备存在问题。
     * @return 频率，单位为赫兹 (Hz)。
     */
    public double getFrequency(){
        if (loopTime != 0){
            return 1000000.0/loopTime;
        }
        else {
            return 0;
        }
    }

    /**
     * 获取X（前进）编码器的原始累计脉冲数（ticks）。
     */
    public int getEncoderX(){return xEncoderValue; }

    /**
     * 获取Y（平移）编码器的原始累计脉冲数（ticks）。
     */
    public int getEncoderY(){return yEncoderValue; }

    /**
     * 获取机器人估算的X（前进）方向位置。
     * @return X位置，单位：毫米 (mm)。
     */
    public double getPosX(){return xPosition; }

    /**
     * 获取机器人估算的Y（平移）方向位置。
     * @return Y位置，单位：毫米 (mm)。
     */
    public double getPosY(){return yPosition; }

    /**
     * 获取机器人估算的航向角。
     * @return 航向角，单位：弧度 (radians)。
     */
    public double getHeading(){return hOrientation;}

    /**
     * 获取机器人估算的X（前进）方向速度。
     * @return X速度，单位：毫米/秒 (mm/s)。
     */
    public double getVelX(){return xVelocity; }

    /**
     * 获取机器人估算的Y（平移）方向速度。
     * @return Y速度，单位：毫米/秒 (mm/s)。
     */
    public double getVelY(){return yVelocity; }

    /**
     * 获取机器人估算的航向角速度。
     * @return 角速度，单位：弧度/秒 (radians/s)。
     */
    public double getHeadingVelocity(){return hVelocity; }

    /**
     * 获取用户设置的X（前进）吊舱的偏移量。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @return X偏移量。
     */
    public float getXOffset(){return readFloat(Register.X_POD_OFFSET);}

    /**
     * 获取用户设置的Y（平移）吊舱的偏移量。
     * <p>
     * <strong>注意：此方法会执行一次独立的I2C读取，为避免性能影响，请勿在控制循环内频繁调用。</strong>
     * @return Y偏移量。
     */
    public float getYOffset(){return readFloat(Register.Y_POD_OFFSET);}

    /**
     * 获取一个包含机器人完整位置信息的`Pose2D`对象。
     * @return 包含X、Y位置和航向角的`Pose2D`对象。
     */
    public Pose2D getPosition(){
        return new Pose2D(DistanceUnit.MM,
                xPosition,
                yPosition,
                AngleUnit.RADIANS,
                hOrientation);
    }



    /**
     * 获取一个包含机器人完整速度信息的`Pose2D`对象。
     * @return 包含X、Y线速度和航向角速度的`Pose2D`对象。速度单位为 毫米/秒 和 弧度/秒。
     */
    public Pose2D getVelocity(){
        return new Pose2D(DistanceUnit.MM,
                xVelocity,
                yVelocity,
                AngleUnit.RADIANS,
                hVelocity);
    }
}
