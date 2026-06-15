package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

@Config
public class Shooter extends SubsystemBase {

    // --- Hardware ---
    private final DcMotorEx shooterL;
    private final DcMotorEx shooterR;
    private final VoltageSensor voltageSensor;
    private final TelemetryData telemetry;

    // --- Motor Constants ---
    public static double MOTOR_CPR = 28;
    public static double SHAFT_CPR = 28;  // encoder counts per motor shaft rev (independent of gearing)
    public static double GEAR_RATIO = 1.0;

    // --- PIDF Constants ---
    public static double Kp = 0.008;
    public static double Kd = 0.0;
    public static double Kf = 0.00055;

    // --- Voltage Compensation Config ---
    public static double VOLTAGE_SENSOR_POLLING_RATE = 40;
    public static double NOMINAL_VOLTAGE = 12.4;

    // --- Shared State ---
    private double currentTargetVelocityTPS = 0;
    private boolean testFullPower = false;

    // --- Voltage State ---
    private double cachedVoltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    // --- Dual Flywheels ---
    private final Flywheel top;
    private final Flywheel bottom;

    public Shooter(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;

        shooterL = hardwareMap.get(DcMotorEx.class, "shooter_t");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooter_b");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // shooter_t MUST have an encoder connected.
        // If wheels ever stop launching straight, one direction may need flipping.

        top = new Flywheel(shooterL);
        bottom = new Flywheel(shooterR);

        cachedVoltage = voltageSensor.getVoltage();
        voltageTimer.reset();
    }

    @Override
    public void periodic() {
        if (voltageTimer.milliseconds() > (1000 / VOLTAGE_SENSOR_POLLING_RATE)) {
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        if (testFullPower) {
            shooterL.setPower(1.0);
            shooterR.setPower(1.0);
            telemetry.addData("Shooter Target (RPM)", "FULL POWER TEST");
            telemetry.addData("Top Actual (RPM)", ticksToRPM(shooterL.getVelocity()));
            telemetry.addData("Bot Actual (RPM)", ticksToRPM(shooterR.getVelocity()));
            telemetry.addData("Top Power", 1.0);
            telemetry.addData("Bot Power", 1.0);
            telemetry.addData("Shooter Voltage", cachedVoltage);
            return;
        }

        double errorTopTPS = top.update(currentTargetVelocityTPS, cachedVoltage);
        double errorBottomTPS = bottom.update(currentTargetVelocityTPS, cachedVoltage);

        telemetry.addData("Shooter Target (RPM)", ticksToRPM(currentTargetVelocityTPS));
        telemetry.addData("Top Actual (RPM)", ticksToRPM(top.getVelocityTPS()));
        telemetry.addData("Bot Actual (RPM)", ticksToRPM(bottom.getVelocityTPS()));
        telemetry.addData("Top Power", top.getPower());
        telemetry.addData("Bot Power", bottom.getPower());
        telemetry.addData("Shooter Voltage", cachedVoltage);
    }

    public void setTargetVelocity(double rpm) {
        currentTargetVelocityTPS = rpmToTicks(rpm);
    }

    public double getError() {
        double eTop = top.getLastErrorTPS();
        double eBot = bottom.getLastErrorTPS();
        double larger = Math.abs(eTop) > Math.abs(eBot) ? eTop : eBot;
        return ticksToRPM(larger);
    }

    public double getTargetVelocity() {
        return ticksToRPM(currentTargetVelocityTPS);
    }

    public void setTestFullPower(boolean on) {
        testFullPower = on;
    }

    public double getRawShaftRpmL() {
        return (shooterL.getVelocity() / SHAFT_CPR) * 60.0;
    }

    public double getRawShaftRpmR() {
        return (shooterR.getVelocity() / SHAFT_CPR) * 60.0;
    }

    // --- Optional diagnostic helpers (additive, do not break anything) ---

    public double getErrorTop() {
        return ticksToRPM(top.getLastErrorTPS());
    }

    public double getErrorBottom() {
        return ticksToRPM(bottom.getLastErrorTPS());
    }

    public double getTopRPM() {
        return ticksToRPM(top.getVelocityTPS());
    }

    public double getBottomRPM() {
        return ticksToRPM(bottom.getVelocityTPS());
    }

    // --- Helper Conversion Methods ---

    private double rpmToTicks(double rpm) {
        return (rpm / 60.0) * (MOTOR_CPR * GEAR_RATIO);
    }

    private double ticksToRPM(double tps) {
        return (tps / (MOTOR_CPR * GEAR_RATIO)) * 60.0;
    }

    // ---------------------------------------------------------------------------
    // Flywheel — private inner class encapsulating one motor's PID loop
    // ---------------------------------------------------------------------------

    private static class Flywheel {
        private final DcMotorEx motor;
        private double lastError = 0;
        private double currentVelocityTPS = 0;
        private double currentPower = 0;
        private final ElapsedTime pidTimer = new ElapsedTime();

        Flywheel(DcMotorEx motor) {
            this.motor = motor;
        }

        /**
         * Run one iteration of the PID + voltage-compensated feedforward loop.
         * @return the error (targetTPS - actualTPS) for this wheel.
         */
        double update(double targetTPS, double cachedVoltage) {
            currentVelocityTPS = motor.getVelocity();

            double error = targetTPS - currentVelocityTPS;

            double dt = pidTimer.seconds();
            pidTimer.reset();

            double pTerm = Kp * error;

            double derivative = dt > 1e-9 ? (error - lastError) / dt : 0;
            double dTerm = Kd * derivative;

            double voltageScale = cachedVoltage / NOMINAL_VOLTAGE;
            if (voltageScale < 0.5) voltageScale = 0.5;

            double fTerm = (Kf / voltageScale) * targetTPS;

            double power = pTerm + dTerm + fTerm + 0.02;

            lastError = error;

            if (Math.abs(targetTPS) < 10) {
                power = 0.0;
            }
            currentPower = Math.max(-1.0, Math.min(1.0, power));

            motor.setPower(currentPower);
            return error;
        }

        double getLastErrorTPS() {
            return lastError;
        }

        double getVelocityTPS() {
            return currentVelocityTPS;
        }

        double getPower() {
            return currentPower;
        }
    }
}