package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Led extends SubsystemBase {

    private final Servo ledController;
    private Servo led2Controller;
    private RobotState targetState = RobotState.OFF;

    // For animations
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isAnimating = false;

    /**
     * Enum mapping robot states to goBILDA Servo PWM values.
     * Mappings based on goBILDA 3110-0002-0001 Product Insight #4.
     */
    public enum RobotState {
        OFF(0.0),             // Black

        // Solid Colors (from chart)
        SHOOTER_IDLE(0.28),   // Red
        SHOOTER_WARMING_UP(0.611), // Blue (Chart says Green is 0.5, Blue is 0.611. Adjusted below)
        SHOOTER_READY(0.480),  // Green (Chart says Sage 0.444, Green 0.5)

        // Animation States
        RGB_CYCLE(0.0);

        public final double pwm;

        RobotState(double pwm) {
            this.pwm = pwm;
        }
    }

    public Led(HardwareMap hardwareMap) {
        ledController = hardwareMap.get(Servo.class, "led");
        try {
            led2Controller = hardwareMap.get(Servo.class, "led2");
        } catch (Exception e) {
            led2Controller = null;
        }
        setState(RobotState.RGB_CYCLE);
    }

    public void setState(RobotState state) {
        // Only reset timer if we are switching INTO an animation from a non-animation
        boolean newIsAnimating = (state == RobotState.RGB_CYCLE);

        if (newIsAnimating && !isAnimating) {
            timer.reset();
        }

        this.targetState = state;
        this.isAnimating = newIsAnimating;
    }
    public RobotState getState() {
        return targetState;
    }

    @Override
    public void periodic() {
        double indicator1Pwm;
        if (PrismConfig.Indicator1_Mode == PrismConfig.IndicatorMode.STATIC) {
            indicator1Pwm = pwmForStaticColor(PrismConfig.Indicator1_StaticColor);
        } else if (isAnimating) {
            indicator1Pwm = handleAnimation();
        } else {
            indicator1Pwm = targetState.pwm;
        }
        writeHardware(indicator1Pwm);

        if (led2Controller != null) {
            double indicator2Pwm;
            if (PrismConfig.Indicator2_Mode == PrismConfig.IndicatorMode.STATIC) {
                indicator2Pwm = pwmForStaticColor(PrismConfig.Indicator2_StaticColor);
            } else {
                indicator2Pwm = indicator1Pwm;
            }
            led2Controller.setPosition(indicator2Pwm);
        }
    }

    private double handleAnimation() {
        double time = timer.seconds();

        if (targetState == RobotState.RGB_CYCLE) {
            double minPwm = 0.28; // Red
            double maxPwm = 0.72; // Violet
            double cycleTime = 4.0;

            double input = (time % cycleTime) / cycleTime;

            double phase;
            if (input <= 0.5) {
                phase = input * 2.0;
            } else {
                phase = (1.0 - input) * 2.0;
            }

            return minPwm + (phase * (maxPwm - minPwm));
        }

        return targetState.pwm;
    }

    private void writeHardware(double pwm) {
        ledController.setPosition(pwm);
    }

    private static double pwmForStaticColor(PrismConfig.IndicatorStaticColor color) {
        switch (color) {
            case RED:    return 0.28;
            case ORANGE: return 0.36;
            case SAGE:   return 0.444;
            case GREEN:  return 0.480;
            case BLUE:   return 0.611;
            case VIOLET: return 0.72;
            case WHITE:  return 1.0;
            default:     return 0.0;
        }
    }
}