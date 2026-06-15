package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

@Configurable
public class PrismConfig {

    @IgnoreConfigurable
    public static final PrismConfig INSTANCE = new PrismConfig();

    public enum StripAnim {
        OFF,
        SOLID,
        BLINK,
        PULSE,
        RAINBOW,
        SPARKLE
    }

    public enum IndicatorMode {
        STATUS,
        STATIC
    }

    public enum IndicatorStaticColor {
        BLACK,
        RED,
        ORANGE,
        SAGE,
        GREEN,
        BLUE,
        VIOLET,
        WHITE
    }

    public enum Direction {
        FORWARD,
        BACKWARD
    }

    // ===== STRIP 0 (pixels 0-5) =====
    public static StripAnim Strip0_Anim = StripAnim.OFF;
    public static int Strip0_Brightness = 100;
    public static int Strip0_StartIdx = 0;
    public static int Strip0_StopIdx = 5;
    public static int Strip0_Color1_R = 255;
    public static int Strip0_Color1_G = 0;
    public static int Strip0_Color1_B = 0;
    public static int Strip0_Color2_R = 0;
    public static int Strip0_Color2_G = 0;
    public static int Strip0_Color2_B = 255;
    public static float Strip0_Speed = 0.5f;
    public static int Strip0_Period = 1000;
    public static Direction Strip0_Direction = Direction.FORWARD;

    // ===== STRIP 1 (pixels 6-11) =====
    public static StripAnim Strip1_Anim = StripAnim.OFF;
    public static int Strip1_Brightness = 100;
    public static int Strip1_StartIdx = 6;
    public static int Strip1_StopIdx = 11;
    public static int Strip1_Color1_R = 0;
    public static int Strip1_Color1_G = 255;
    public static int Strip1_Color1_B = 0;
    public static int Strip1_Color2_R = 0;
    public static int Strip1_Color2_G = 0;
    public static int Strip1_Color2_B = 255;
    public static float Strip1_Speed = 0.5f;
    public static int Strip1_Period = 1000;
    public static Direction Strip1_Direction = Direction.FORWARD;

    // ===== STRIP 2 (pixels 12-23) =====
    public static StripAnim Strip2_Anim = StripAnim.OFF;
    public static int Strip2_Brightness = 100;
    public static int Strip2_StartIdx = 12;
    public static int Strip2_StopIdx = 23;
    public static int Strip2_Color1_R = 0;
    public static int Strip2_Color1_G = 0;
    public static int Strip2_Color1_B = 255;
    public static int Strip2_Color2_R = 255;
    public static int Strip2_Color2_G = 0;
    public static int Strip2_Color2_B = 0;
    public static float Strip2_Speed = 0.5f;
    public static int Strip2_Period = 1000;
    public static Direction Strip2_Direction = Direction.FORWARD;

    // ===== STRIP 3 (pixels 24-35) =====
    public static StripAnim Strip3_Anim = StripAnim.OFF;
    public static int Strip3_Brightness = 100;
    public static int Strip3_StartIdx = 24;
    public static int Strip3_StopIdx = 35;
    public static int Strip3_Color1_R = 255;
    public static int Strip3_Color1_G = 255;
    public static int Strip3_Color1_B = 255;
    public static int Strip3_Color2_R = 0;
    public static int Strip3_Color2_G = 0;
    public static int Strip3_Color2_B = 0;
    public static float Strip3_Speed = 0.5f;
    public static int Strip3_Period = 1000;
    public static Direction Strip3_Direction = Direction.FORWARD;

    // ===== Indicator Servo PWM Constants =====
    @IgnoreConfigurable
    public static final double INDICATOR_RED_POS = 0.28;
    @IgnoreConfigurable
    public static final double INDICATOR_GREEN_POS = 0.480;
    @IgnoreConfigurable
    public static final double INDICATOR_ON_POS = 0.48;

    // ===== INDICATOR 1 (existing "led" servo) =====
    public static IndicatorMode Indicator1_Mode = IndicatorMode.STATUS;
    public static IndicatorStaticColor Indicator1_StaticColor = IndicatorStaticColor.RED;

    // ===== INDICATOR 2 (new servo) =====
    public static IndicatorMode Indicator2_Mode = IndicatorMode.STATUS;
    public static IndicatorStaticColor Indicator2_StaticColor = IndicatorStaticColor.GREEN;
}
