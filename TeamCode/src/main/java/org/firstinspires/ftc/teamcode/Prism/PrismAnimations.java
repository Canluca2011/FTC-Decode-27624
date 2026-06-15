package org.firstinspires.ftc.teamcode.Prism;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.TimeUnit;

public class PrismAnimations {

    public enum AnimationType {
        NONE(0),
        SOLID(1),
        BLINK(2),
        PULSE(3),
        SINE_WAVE(4),
        DROID_SCAN(5),
        RAINBOW(6),
        SNAKES(7),
        RANDOM(8),
        SPARKLE(9),
        SINGLE_FILL(10),
        RAINBOW_SNAKES(11),
        POLICE_LIGHTS(12);

        public final int id;

        AnimationType(int id) {
            this.id = id;
        }
    }

    public static abstract class AnimationBase {
        protected final AnimationType animationType;
        protected int brightness = 100;
        protected int startIndex = 0;
        protected int stopIndex = 255;
        public LayerHeight layerHeight;

        protected AnimationBase(AnimationType type) {
            this.animationType = type;
        }

        protected AnimationBase(AnimationType type, int brightness) {
            this(type);
            this.brightness = Math.min(brightness, 100);
        }

        protected AnimationBase(AnimationType type, int startIndex, int stopIndex) {
            this(type);
            this.startIndex = Math.min(startIndex, 255);
            this.stopIndex = Math.min(stopIndex, 255);
        }

        protected AnimationBase(AnimationType type, int brightness, int startIndex, int stopIndex) {
            this(type, brightness);
            this.startIndex = Math.min(startIndex, 255);
            this.stopIndex = Math.min(stopIndex, 255);
        }

        public int getBrightness() {
            return brightness;
        }

        public int getStartIndex() {
            return startIndex;
        }

        public int getStopIndex() {
            return stopIndex;
        }

        public void setBrightness(int brightness) {
            this.brightness = Math.min(brightness, 100);
        }

        public void setStartIndex(int startIndex) {
            this.startIndex = Math.min(startIndex, 255);
        }

        public void setStopIndex(int stopIndex) {
            this.stopIndex = Math.min(stopIndex, 255);
        }

        public void setIndexes(int startIndex, int stopIndex) {
            this.startIndex = startIndex;
            this.stopIndex = stopIndex;
        }

        protected abstract void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient);

        protected void updateAnimationOverI2C(I2cDeviceSynchSimple deviceClient, boolean isInsertingAnimation) {
            if (isInsertingAnimation)
                deviceClient.write(layerHeight.register.address, GetByteArray(0, animationType.id));
            deviceClient.write(layerHeight.register.address, GetByteArray(1, brightness));
            deviceClient.write(layerHeight.register.address, GetByteArray(2, startIndex));
            deviceClient.write(layerHeight.register.address, GetByteArray(3, stopIndex));
            updateAnimationSpecificValuesOverI2C(deviceClient);
        }

        protected byte[] floatToByteArray(float value, ByteOrder byteOrder) {
            return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
        }

        protected byte[] GetByteArray(int subRegister, Color color) {
            return new byte[]{
                    (byte) Math.max(0, Math.min(subRegister, 12)),
                    (byte) color.red,
                    (byte) color.green,
                    (byte) color.blue
            };
        }

        protected byte[] GetByteArray(int subRegister, int data) {
            byte[] packet = TypeConversion.intToByteArray(data, ByteOrder.LITTLE_ENDIAN);
            return new byte[]{
                    (byte) Math.max(0, Math.min(subRegister, 12)),
                    packet[0], packet[1], packet[2], packet[3]
            };
        }

        protected byte[] GetByteArray(int subRegister, float data) {
            byte[] packet = floatToByteArray(data, ByteOrder.LITTLE_ENDIAN);
            return new byte[]{
                    (byte) Math.max(0, Math.min(subRegister, 12)),
                    packet[0], packet[1], packet[2], packet[3]
            };
        }

        protected byte[] GetByteArray(int subRegister, byte data) {
            return new byte[]{
                    (byte) subRegister, data
            };
        }

        protected byte[] GetByteArray(int subRegister, Direction direction) {
            return new byte[]{
                    (byte) Math.max(0, Math.min(subRegister, 12)),
                    (byte) (direction == Direction.Forward ? 1 : 0)
            };
        }

        protected byte[] GetByteArray(int subRegister, Color... colors) {
            byte[] array = new byte[1 + (colors.length * 3)];
            array[0] = (byte) Math.max(0, Math.min(subRegister, 12));
            for (int i = 0; i < colors.length; i++) {
                array[1 + (i * 3)] = (byte) colors[i].red;
                array[1 + (i * 3 + 1)] = (byte) colors[i].green;
                array[1 + (i * 3 + 2)] = (byte) colors[i].blue;
            }
            return array;
        }
    }

    public static class Solid extends AnimationBase {
        private Color primaryColor = Color.RED;

        public Solid() {
            super(AnimationType.SOLID);
        }

        public Solid(Color primaryColor) {
            super(AnimationType.SOLID);
            this.primaryColor = primaryColor;
        }

        public Solid(Color primaryColor, int brightness) {
            super(AnimationType.SOLID, brightness);
            this.primaryColor = primaryColor;
        }

        public void setPrimaryColor(Color color) {
            primaryColor = color;
        }

        public Color getPrimaryColor() {
            return primaryColor;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
        }
    }

    public static class Blink extends AnimationBase {
        private Color primaryColor = Color.BLUE;
        private Color secondaryColor = Color.RED;
        private int period = 2000;
        private int primaryColorPeriod = 1000;

        public Blink() {
            super(AnimationType.BLINK);
        }

        public Blink(Color primaryColor) {
            this();
            this.primaryColor = primaryColor;
        }

        public void setPeriod(int period) {
            this.period = period;
        }

        public void setPrimaryColor(Color color) {
            primaryColor = color;
        }

        public void setSecondaryColor(Color color) {
            secondaryColor = color;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, secondaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, period));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, primaryColorPeriod));
        }
    }

    public static class Pulse extends AnimationBase {
        private Color primaryColor = Color.GREEN;
        private Color secondaryColor = Color.RED;
        private int period = 1000;

        public Pulse() {
            super(AnimationType.PULSE);
        }

        public Pulse(Color primaryColor) {
            this();
            this.primaryColor = primaryColor;
        }

        public void setPeriod(int period) {
            this.period = period;
        }

        public void setPrimaryColor(Color color) {
            primaryColor = color;
        }

        public void setSecondaryColor(Color color) {
            secondaryColor = color;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, secondaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, period));
        }
    }

    public static class Rainbow extends AnimationBase {
        private float startHue = 0.0f;
        private float stopHue = 360.0f;
        private float speed = 0.50f;
        private int repeatAfter = 25;
        private Direction direction = Direction.Forward;

        public Rainbow() {
            super(AnimationType.RAINBOW);
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setStartHue(float startHue) {
            this.startHue = startHue;
        }

        public void setStopHue(float stopHue) {
            this.stopHue = stopHue;
        }

        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        public void setRepeatAfter(int repeatAfter) {
            this.repeatAfter = repeatAfter;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, startHue));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, stopHue));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, speed));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, direction));
            deviceClient.write(layerHeight.register.address, GetByteArray(9, (byte) repeatAfter));
        }
    }

    public static class SineWave extends AnimationBase {
        private Color secondaryColor = Color.BLUE;
        private Color primaryColor = Color.RED;
        private Direction direction = Direction.Forward;
        private float offset = 0.5f;
        private float speed = 0.5f;
        private int period = 1000;

        public SineWave() {
            super(AnimationType.SINE_WAVE);
        }

        public void setPeriod(int period) {
            this.period = period;
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setOffset(float offset) {
            this.offset = offset;
        }

        public void setPrimaryColor(Color color) {
            primaryColor = color;
        }

        public void setSecondaryColor(Color color) {
            secondaryColor = color;
        }

        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, secondaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, period));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, direction));
            deviceClient.write(layerHeight.register.address, GetByteArray(8, offset));
            deviceClient.write(layerHeight.register.address, GetByteArray(9, speed));
        }
    }

    public static class DroidScan extends AnimationBase {
        private Color primaryColor = Color.RED;
        private Color secondaryColor = Color.TRANSPARENT;
        private float speed = 0.4f;
        private int eyeWidth = 3;
        private int trailWidth = 3;

        public DroidScan() {
            super(AnimationType.DROID_SCAN);
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setPrimaryColor(Color color) {
            primaryColor = color;
        }

        public void setSecondaryColor(Color color) {
            secondaryColor = color;
        }

        public void setEyeWidth(int eyeWidth) {
            this.eyeWidth = eyeWidth;
        }

        public void setTrailWidth(int trailWidth) {
            this.trailWidth = trailWidth;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, secondaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, speed));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, (byte) eyeWidth));
            deviceClient.write(layerHeight.register.address, GetByteArray(8, (byte) trailWidth));
        }
    }

    public static class Snakes extends AnimationBase {
        private Color[] colors = {Color.RED, Color.WHITE, Color.BLUE};
        private int snakeLength = 5;
        private int spacingBetween = 2;
        private int repeatAfter = 15;
        private Color backgroundColor = Color.TRANSPARENT;
        private float speed = 0.50f;
        private Direction direction = Direction.Backward;

        public Snakes() {
            super(AnimationType.SNAKES);
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        public void setColors(Color... colors) {
            this.colors = colors;
        }

        public void setSnakeLength(int snakeLength) {
            this.snakeLength = snakeLength;
        }

        public void setSpacingBetween(int spacingBetween) {
            this.spacingBetween = spacingBetween;
        }

        public void setRepeatAfter(int repeatAfter) {
            this.repeatAfter = repeatAfter;
        }

        public void setBackgroundColor(Color backgroundColor) {
            this.backgroundColor = backgroundColor;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, (byte) colors.length));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, colors));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, speed));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, direction));
            deviceClient.write(layerHeight.register.address, GetByteArray(8, (byte) spacingBetween));
            deviceClient.write(layerHeight.register.address, GetByteArray(9, (byte) repeatAfter));
            deviceClient.write(layerHeight.register.address, GetByteArray(10, backgroundColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(11, (byte) snakeLength));
        }
    }

    public static class Random extends AnimationBase {
        private float startHue = 0f;
        private float stopHue = 360f;
        private float speed = 0.1f;

        public Random() {
            super(AnimationType.RANDOM);
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setStartHue(float startHue) {
            this.startHue = startHue;
        }

        public void setStopHue(float stopHue) {
            this.stopHue = stopHue;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, startHue));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, stopHue));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, speed));
        }
    }

    public static class Sparkle extends AnimationBase {
        private Color primaryColor = Color.WHITE;
        private Color secondaryColor = Color.TRANSPARENT;
        private int sparkleProbability = 16;
        private int period = 100;

        public Sparkle() {
            super(AnimationType.SPARKLE);
        }

        public void setPeriod(int period) {
            this.period = period;
        }

        public void setPrimaryColor(Color primaryColor) {
            this.primaryColor = primaryColor;
        }

        public void setSecondaryColor(Color secondaryColor) {
            this.secondaryColor = secondaryColor;
        }

        public void setSparkleProbability(int sparkleProbability) {
            this.sparkleProbability = sparkleProbability;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, primaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, secondaryColor));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, period));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, (byte) sparkleProbability));
        }
    }

    public static class SingleFill extends AnimationBase {
        private Color[] colors = {Color.WHITE, Color.GREEN, Color.BLUE};
        private int period = 750;
        private float speed = 0.5f;
        private int pixelLength = 3;
        private Direction direction = Direction.Backward;

        public SingleFill() {
            super(AnimationType.SINGLE_FILL);
        }

        public void setPeriod(int period) {
            this.period = period;
        }

        public void setSpeed(float speed) {
            this.speed = speed;
        }

        public void setColors(Color... colors) {
            this.colors = colors;
        }

        public void setPixelLength(int pixelLength) {
            this.pixelLength = pixelLength;
        }

        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        @Override
        protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
            deviceClient.write(layerHeight.register.address, GetByteArray(4, (byte) colors.length));
            deviceClient.write(layerHeight.register.address, GetByteArray(5, colors));
            deviceClient.write(layerHeight.register.address, GetByteArray(6, period));
            deviceClient.write(layerHeight.register.address, GetByteArray(7, direction));
            deviceClient.write(layerHeight.register.address, GetByteArray(8, (byte) pixelLength));
            deviceClient.write(layerHeight.register.address, GetByteArray(9, speed));
        }
    }
}
