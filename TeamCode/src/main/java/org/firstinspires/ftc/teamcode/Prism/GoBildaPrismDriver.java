package org.firstinspires.ftc.teamcode.Prism;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import static com.qualcomm.robotcore.util.TypeConversion.unsignedByteToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(
        name = "goBILDA Prism RGB LED Driver",
        xmlTag = "goBILDAPrism",
        description = "Prism RGB LED Driver (6-30V Input, I2C / PWM Control)"
)
public class GoBildaPrismDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    public static final byte DEFAULT_ADDRESS = 0x38;
    private static final int MAXIMUM_NUMBER_OF_ANIMATIONS = 10;

    private final PrismAnimations.AnimationBase[] animations = new PrismAnimations.AnimationBase[MAXIMUM_NUMBER_OF_ANIMATIONS];

    public enum LayerHeight {
        LAYER_0(Register.ANIMATION_SLOT_0),
        LAYER_1(Register.ANIMATION_SLOT_1),
        LAYER_2(Register.ANIMATION_SLOT_2),
        LAYER_3(Register.ANIMATION_SLOT_3),
        LAYER_4(Register.ANIMATION_SLOT_4),
        LAYER_5(Register.ANIMATION_SLOT_5),
        LAYER_6(Register.ANIMATION_SLOT_6),
        LAYER_7(Register.ANIMATION_SLOT_7),
        LAYER_8(Register.ANIMATION_SLOT_8),
        LAYER_9(Register.ANIMATION_SLOT_9),
        DISABLED(Register.NULL);

        final Register register;
        final int index;

        LayerHeight(Register register) {
            this.register = register;
            if (register == Register.NULL)
                this.index = -1;
            else
                this.index = register.address - Register.ANIMATION_SLOT_0.address;
        }
    }

    public enum Artboard {
        ARTBOARD_0(0, 0),
        ARTBOARD_1(1, 1),
        ARTBOARD_2(2, 2),
        ARTBOARD_3(3, 3),
        ARTBOARD_4(4, 4),
        ARTBOARD_5(5, 5),
        ARTBOARD_6(6, 6),
        ARTBOARD_7(7, 7);

        final byte bitmask;
        public final int index;

        Artboard(int val, int index) {
            this.bitmask = (byte) (1 << val);
            this.index = index;
        }
    }

    enum RegisterType {
        INT8(1, 255),
        INT16(2, 65535),
        INT24(3, 16777215),
        INT32(4, 2147483647);

        final int lengthBytes;
        final int maxValue;

        RegisterType(int lengthBytes, int maxValue) {
            this.lengthBytes = lengthBytes;
            this.maxValue = maxValue;
        }
    }

    enum Register {
        DEVICE_ID(0, RegisterType.INT8, RegisterAccess.READ_ONLY),
        FIRMWARE_VERSION(1, RegisterType.INT24, RegisterAccess.READ_ONLY),
        HARDWARE_VERSION(2, RegisterType.INT16, RegisterAccess.READ_ONLY),
        POWER_CYCLE_COUNT(3, RegisterType.INT32, RegisterAccess.READ_ONLY),
        RUNTIME_IN_MINUTES(4, RegisterType.INT32, RegisterAccess.READ_ONLY),
        STATUS(5, RegisterType.INT32, RegisterAccess.READ_ONLY),
        CONTROL(6, RegisterType.INT32, RegisterAccess.WRITE_ONLY),
        ARTBOARD_CONTROL(7, RegisterType.INT32, RegisterAccess.WRITE_ONLY),
        ANIMATION_SLOT_0(8, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_1(9, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_2(10, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_3(11, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_4(12, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_5(13, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_6(14, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_7(15, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_8(16, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_9(17, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        NULL(18, RegisterType.INT8, RegisterAccess.READ_ONLY);

        final int address;
        final RegisterType registerType;
        final RegisterAccess registerAccess;

        Register(int address, RegisterType registerType, RegisterAccess registerAccess) {
            this.address = address;
            this.registerType = registerType;
            this.registerAccess = registerAccess;
        }
    }

    enum RegisterAccess {
        READ_ONLY,
        WRITE_ONLY,
        READ_AND_WRITE
    }

    private int readInt(Register register) {
        return byteArrayToInt(deviceClient.read(register.address, register.registerType.lengthBytes), ByteOrder.LITTLE_ENDIAN);
    }

    public GoBildaPrismDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.GoBilda;
    }

    @Override
    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA Prism RGB LED Driver";
    }

    public int getDeviceID() {
        byte[] packet = deviceClient.read(Register.DEVICE_ID.address, Register.DEVICE_ID.registerType.lengthBytes);
        return packet[0];
    }

    public int[] getFirmwareVersion() {
        byte[] packet = deviceClient.read(Register.FIRMWARE_VERSION.address, Register.FIRMWARE_VERSION.registerType.lengthBytes);
        return new int[]{packet[2], packet[1], packet[0]};
    }

    public String getFirmwareVersionString() {
        int[] v = getFirmwareVersion();
        return String.format("%d.%d.%d", v[0], v[1], v[2]);
    }

    public int[] getHardwareVersion() {
        byte[] packet = deviceClient.read(Register.HARDWARE_VERSION.address, Register.HARDWARE_VERSION.registerType.lengthBytes);
        return new int[]{packet[1], packet[0]};
    }

    public String getHardwareVersionString() {
        int[] v = getHardwareVersion();
        return String.format("%d.%d", v[0], v[1]);
    }

    public int getPowerCycleCount() {
        return readInt(Register.POWER_CYCLE_COUNT);
    }

    public long getRunTime(TimeUnit timeUnit) {
        return timeUnit.convert(readInt(Register.RUNTIME_IN_MINUTES), TimeUnit.MINUTES);
    }

    public int getNumberOfLEDs() {
        byte[] packet = deviceClient.read(Register.STATUS.address, Register.STATUS.registerType.lengthBytes);
        return unsignedByteToInt(packet[0]);
    }

    public int getCurrentFPS() {
        byte[] inputPacket = deviceClient.read(Register.STATUS.address, Register.STATUS.registerType.lengthBytes);
        byte[] outputPacket = new byte[4];
        outputPacket[0] = inputPacket[1];
        outputPacket[1] = inputPacket[2];
        return byteArrayToInt(outputPacket, ByteOrder.LITTLE_ENDIAN);
    }

    public boolean insertAnimation(LayerHeight height, PrismAnimations.AnimationBase animation) {
        if (height == LayerHeight.DISABLED || animation == null) return false;
        animations[height.index] = animation;
        animations[height.index].layerHeight = height;
        return true;
    }

    public boolean insertAndUpdateAnimation(LayerHeight height, PrismAnimations.AnimationBase animation) {
        if (insertAnimation(height, animation))
            return updateAnimationFromIndex(height, true);
        return false;
    }

    public boolean updateAllAnimations() {
        for (int i = 0; i < MAXIMUM_NUMBER_OF_ANIMATIONS; i++) {
            if (animations[i] != null && animations[i].layerHeight != LayerHeight.DISABLED)
                animations[i].updateAnimationOverI2C(deviceClient, false);
        }
        return true;
    }

    public boolean updateAnimationFromIndex(LayerHeight height) {
        return updateAnimationFromIndex(height, false);
    }

    public boolean updateAnimationFromIndex(LayerHeight height, boolean isBeingInserted) {
        if (height == LayerHeight.DISABLED || animations[height.index] == null) return false;
        boolean animationEnabled = animations[height.index].layerHeight != LayerHeight.DISABLED;
        if (animationEnabled)
            animations[height.index].updateAnimationOverI2C(deviceClient, isBeingInserted);
        return animationEnabled;
    }

    public void clearAllAnimations() {
        byte[] packet = TypeConversion.intToByteArray(1 << 25, ByteOrder.LITTLE_ENDIAN);
        deviceClient.write(Register.CONTROL.address, packet);
        Arrays.fill(animations, null);
    }

    public void setTargetFPS(int targetFPS) {
        int bounded = Math.max(0, Math.min(targetFPS, 0x7FFF));
        int command = (1 << 15) | bounded;
        byte[] packet = TypeConversion.intToByteArray(command, ByteOrder.LITTLE_ENDIAN);
        deviceClient.write(Register.CONTROL.address, packet);
    }

    public void setStripLength(int stripLength) {
        int bounded = Math.max(0, Math.min(stripLength, 0xFF));
        int command = (1 << 24) | (bounded << 16);
        byte[] packet = TypeConversion.intToByteArray(command, ByteOrder.LITTLE_ENDIAN);
        deviceClient.write(Register.CONTROL.address, packet);
    }

    public void saveCurrentAnimationsToArtboard(Artboard artboard) {
        deviceClient.write(Register.ARTBOARD_CONTROL.address, new byte[]{artboard.bitmask, 0, 0, 0});
    }

    public void loadAnimationsFromArtboard(Artboard artboard) {
        deviceClient.write(Register.ARTBOARD_CONTROL.address, new byte[]{0, artboard.bitmask, 0, 0});
    }

    public void setDefaultBootArtboard(Artboard artboard) {
        deviceClient.write(Register.ARTBOARD_CONTROL.address, new byte[]{0, 0, artboard.bitmask, 0});
    }

    public void enableDefaultBootArtboard(boolean enable) {
        deviceClient.write(Register.ARTBOARD_CONTROL.address, new byte[]{0, 0, 0, enable ? (byte) 0b00000001 : (byte) 0b00000010});
    }
}
