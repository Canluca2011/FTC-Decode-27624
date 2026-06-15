package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.Objects;

public class PrismSubsystem extends SubsystemBase {

    private final GoBildaPrismDriver prism;
    private final boolean prismAvailable;
    private TelemetryData telemetry;

    private int prevConfigHash = 0;
    private int loopCount = 0;

    public PrismSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public PrismSubsystem(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        GoBildaPrismDriver d;
        boolean ok;
        try {
            d = hardwareMap.get(GoBildaPrismDriver.class, "prism");
            d.initialize();
            ok = true;
        } catch (Exception e) {
            d = null;
            ok = false;
            if (telemetry != null) {
                telemetry.addData("Prism", "NOT FOUND: " + e.getMessage());
            }
        }
        prism = d;
        prismAvailable = ok;
    }

    @Override
    public void periodic() {
        if (!prismAvailable || prism == null) return;

        int hash = computeConfigHash();
        if (hash == prevConfigHash) return;
        prevConfigHash = hash;

        for (int i = 0; i < 4; i++) {
            if (stripAnim(i) == PrismConfig.StripAnim.OFF) {
                continue;
            }
            PrismAnimations.AnimationBase anim = buildAnimation(i);
            if (anim != null) {
                LayerHeight layer = LayerHeight.values()[i];
                prism.insertAndUpdateAnimation(layer, anim);
            }
        }

        if (telemetry != null) {
            telemetry.addData("Prism", "applied config hash=" + hash);
        }
    }

    public boolean isAvailable() {
        return prismAvailable;
    }

    public GoBildaPrismDriver getDriver() {
        return prism;
    }

    private int computeConfigHash() {
        return Objects.hash(
                PrismConfig.Strip0_Anim, PrismConfig.Strip0_Brightness,
                PrismConfig.Strip0_StartIdx, PrismConfig.Strip0_StopIdx,
                PrismConfig.Strip0_Color1_R, PrismConfig.Strip0_Color1_G, PrismConfig.Strip0_Color1_B,
                PrismConfig.Strip0_Color2_R, PrismConfig.Strip0_Color2_G, PrismConfig.Strip0_Color2_B,
                PrismConfig.Strip0_Speed, PrismConfig.Strip0_Period, PrismConfig.Strip0_Direction,
                PrismConfig.Strip1_Anim, PrismConfig.Strip1_Brightness,
                PrismConfig.Strip1_StartIdx, PrismConfig.Strip1_StopIdx,
                PrismConfig.Strip1_Color1_R, PrismConfig.Strip1_Color1_G, PrismConfig.Strip1_Color1_B,
                PrismConfig.Strip1_Color2_R, PrismConfig.Strip1_Color2_G, PrismConfig.Strip1_Color2_B,
                PrismConfig.Strip1_Speed, PrismConfig.Strip1_Period, PrismConfig.Strip1_Direction,
                PrismConfig.Strip2_Anim, PrismConfig.Strip2_Brightness,
                PrismConfig.Strip2_StartIdx, PrismConfig.Strip2_StopIdx,
                PrismConfig.Strip2_Color1_R, PrismConfig.Strip2_Color1_G, PrismConfig.Strip2_Color1_B,
                PrismConfig.Strip2_Color2_R, PrismConfig.Strip2_Color2_G, PrismConfig.Strip2_Color2_B,
                PrismConfig.Strip2_Speed, PrismConfig.Strip2_Period, PrismConfig.Strip2_Direction,
                PrismConfig.Strip3_Anim, PrismConfig.Strip3_Brightness,
                PrismConfig.Strip3_StartIdx, PrismConfig.Strip3_StopIdx,
                PrismConfig.Strip3_Color1_R, PrismConfig.Strip3_Color1_G, PrismConfig.Strip3_Color1_B,
                PrismConfig.Strip3_Color2_R, PrismConfig.Strip3_Color2_G, PrismConfig.Strip3_Color2_B,
                PrismConfig.Strip3_Speed, PrismConfig.Strip3_Period, PrismConfig.Strip3_Direction
        );
    }

    private static PrismConfig.StripAnim stripAnim(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_Anim;
            case 1:  return PrismConfig.Strip1_Anim;
            case 2:  return PrismConfig.Strip2_Anim;
            case 3:  return PrismConfig.Strip3_Anim;
            default: return PrismConfig.StripAnim.OFF;
        }
    }

    private static PrismAnimations.AnimationBase buildAnimation(int idx) {
        PrismConfig.StripAnim type = stripAnim(idx);
        if (type == PrismConfig.StripAnim.OFF) return null;

        int r1 = stripField(idx, 0, 0);
        int g1 = stripField(idx, 0, 1);
        int b1 = stripField(idx, 0, 2);
        int r2 = stripField(idx, 1, 0);
        int g2 = stripField(idx, 1, 1);
        int b2 = stripField(idx, 1, 2);
        Color c1 = new Color(r1, g1, b1);
        Color c2 = new Color(r2, g2, b2);
        float speed = stripSpeed(idx);
        int period = stripPeriod(idx);
        PrismConfig.Direction dir = stripDirection(idx);
        Direction drvDir = (dir == PrismConfig.Direction.FORWARD) ? Direction.Forward : Direction.Backward;

        PrismAnimations.AnimationBase anim;
        switch (type) {
            case SOLID: {
                PrismAnimations.Solid s = new PrismAnimations.Solid(c1);
                anim = s;
                break;
            }
            case BLINK: {
                PrismAnimations.Blink b = new PrismAnimations.Blink(c1);
                b.setSecondaryColor(c2);
                b.setPeriod(period);
                anim = b;
                break;
            }
            case PULSE: {
                PrismAnimations.Pulse p = new PrismAnimations.Pulse(c1);
                p.setSecondaryColor(c2);
                p.setPeriod(period);
                anim = p;
                break;
            }
            case RAINBOW: {
                PrismAnimations.Rainbow r = new PrismAnimations.Rainbow();
                r.setSpeed(speed);
                r.setDirection(drvDir);
                anim = r;
                break;
            }
            case SPARKLE: {
                PrismAnimations.Sparkle s = new PrismAnimations.Sparkle();
                s.setPrimaryColor(c1);
                s.setSecondaryColor(c2);
                s.setPeriod(period);
                anim = s;
                break;
            }
            default:
                return null;
        }

        anim.setBrightness(stripBrightness(idx));
        anim.setIndexes(stripStart(idx), stripStop(idx));
        return anim;
    }

    private static int stripBrightness(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_Brightness;
            case 1:  return PrismConfig.Strip1_Brightness;
            case 2:  return PrismConfig.Strip2_Brightness;
            case 3:  return PrismConfig.Strip3_Brightness;
            default: return 100;
        }
    }

    private static int stripStart(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_StartIdx;
            case 1:  return PrismConfig.Strip1_StartIdx;
            case 2:  return PrismConfig.Strip2_StartIdx;
            case 3:  return PrismConfig.Strip3_StartIdx;
            default: return 0;
        }
    }

    private static int stripStop(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_StopIdx;
            case 1:  return PrismConfig.Strip1_StopIdx;
            case 2:  return PrismConfig.Strip2_StopIdx;
            case 3:  return PrismConfig.Strip3_StopIdx;
            default: return 0;
        }
    }

    private static int stripField(int idx, int colorNum, int component) {
        // colorNum: 0=Color1, 1=Color2
        // component: 0=R, 1=G, 2=B
        switch (idx) {
            case 0:
                if (colorNum == 0) {
                    return component == 0 ? PrismConfig.Strip0_Color1_R :
                           component == 1 ? PrismConfig.Strip0_Color1_G : PrismConfig.Strip0_Color1_B;
                } else {
                    return component == 0 ? PrismConfig.Strip0_Color2_R :
                           component == 1 ? PrismConfig.Strip0_Color2_G : PrismConfig.Strip0_Color2_B;
                }
            case 1:
                if (colorNum == 0) {
                    return component == 0 ? PrismConfig.Strip1_Color1_R :
                           component == 1 ? PrismConfig.Strip1_Color1_G : PrismConfig.Strip1_Color1_B;
                } else {
                    return component == 0 ? PrismConfig.Strip1_Color2_R :
                           component == 1 ? PrismConfig.Strip1_Color2_G : PrismConfig.Strip1_Color2_B;
                }
            case 2:
                if (colorNum == 0) {
                    return component == 0 ? PrismConfig.Strip2_Color1_R :
                           component == 1 ? PrismConfig.Strip2_Color1_G : PrismConfig.Strip2_Color1_B;
                } else {
                    return component == 0 ? PrismConfig.Strip2_Color2_R :
                           component == 1 ? PrismConfig.Strip2_Color2_G : PrismConfig.Strip2_Color2_B;
                }
            case 3:
                if (colorNum == 0) {
                    return component == 0 ? PrismConfig.Strip3_Color1_R :
                           component == 1 ? PrismConfig.Strip3_Color1_G : PrismConfig.Strip3_Color1_B;
                } else {
                    return component == 0 ? PrismConfig.Strip3_Color2_R :
                           component == 1 ? PrismConfig.Strip3_Color2_G : PrismConfig.Strip3_Color2_B;
                }
            default:
                return 0;
        }
    }

    private static float stripSpeed(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_Speed;
            case 1:  return PrismConfig.Strip1_Speed;
            case 2:  return PrismConfig.Strip2_Speed;
            case 3:  return PrismConfig.Strip3_Speed;
            default: return 0.5f;
        }
    }

    private static int stripPeriod(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_Period;
            case 1:  return PrismConfig.Strip1_Period;
            case 2:  return PrismConfig.Strip2_Period;
            case 3:  return PrismConfig.Strip3_Period;
            default: return 1000;
        }
    }

    private static PrismConfig.Direction stripDirection(int idx) {
        switch (idx) {
            case 0:  return PrismConfig.Strip0_Direction;
            case 1:  return PrismConfig.Strip1_Direction;
            case 2:  return PrismConfig.Strip2_Direction;
            case 3:  return PrismConfig.Strip3_Direction;
            default: return PrismConfig.Direction.FORWARD;
        }
    }
}
