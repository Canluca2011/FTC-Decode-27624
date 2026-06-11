package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

// SolversLib Import
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

@Config
public class Shooter extends SubsystemBase {

    // --- Hardware ---
    private final DcMotorEx shooterL;
    private final DcMotorEx shooterR;
    private final VoltageSensor voltageSensor;
    private final TelemetryData telemetry;

    // --- Motor Constants (YOU MUST VERIFY THESE) ---
    // Example: GoBilda Yellow Jacket 1:1 is 28.
    // Example: Rev HD Hex is 28.
    // Example: If you have a 1:1 motor but a 2:1 belt drive on the shooter, ratio is 2.0
    public static double MOTOR_CPR = 28;
    public static double GEAR_RATIO = 1.0;

    // --- PIDF Constants ---
    // Note: Since we are controlling in TPS internally, these values might need
    // retuning if you were previously tuning based on a different unit scale.
    public static double Kp = 0.008; //TODO
    public static double Kd = 0.0;
    public static double Kf = 0.0005;

    // --- Voltage Compensation Config ---
    public static double VOLTAGE_SENSOR_POLLING_RATE = 40;
    public static double NOMINAL_VOLTAGE = 12.4;

    // --- State Variables ---
    private double currentTargetVelocityTPS = 0; // We store target in TPS for the PID
    private double currentVelocityTPS = 0;
    private double lastError = 0;
    private double cachedVoltage = 12.0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;

        shooterL = hardwareMap.get(DcMotorEx.class, "shooter_t");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooter_b");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure Left motor follows the same mode (even if not using its encoder)
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cachedVoltage = voltageSensor.getVoltage();
        pidTimer.reset();
        voltageTimer.reset();
    }

    @Override
    public void periodic() {
        // --- 1. Update Cached Voltage ---
        if (voltageTimer.milliseconds() > (1000 / VOLTAGE_SENSOR_POLLING_RATE)) {
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        // --- 2. Get current velocity (Native TPS) ---
        currentVelocityTPS = shooterR.getVelocity();

        // --- 3. Calculate Error (in TPS) ---
        double error = currentTargetVelocityTPS - currentVelocityTPS;

        // --- 4. Time Delta ---
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // --- 5. PID Calculations ---
        double pTerm = Kp * error;

        double derivative = (error - lastError) / dt;
        double dTerm = Kd * derivative;

        // Voltage Feedforward
        double voltageScale = cachedVoltage / NOMINAL_VOLTAGE;
        if (voltageScale < 0.5) voltageScale = 0.5; // Sanity check

        double adjustedKf = Kf / voltageScale;
        double fTerm = adjustedKf * currentTargetVelocityTPS;

        // --- 6. Final Power ---
        double power = pTerm + dTerm + fTerm + 0.02;

        lastError = error;

        // --- 7. Apply to motors ---
        double safePower = Math.max(-1.0, Math.min(1.0, power));

        if (Math.abs(currentTargetVelocityTPS) < 10) { // Deadzone for stopping
            safePower = 0.0;
        }

        // BUG FIX: You were setting power to 1.0 hardcoded here previously!
        shooterL.setPower(safePower);
        shooterR.setPower(safePower);

        // --- 8. Telemetry (Converted to RPM for you) ---
        telemetry.addData("Shooter Target (RPM)", ticksToRPM(currentTargetVelocityTPS));
        telemetry.addData("Shooter Actual (RPM)", ticksToRPM(currentVelocityTPS));
        telemetry.addData("Shooter Power", safePower);
        telemetry.addData("Shooter Voltage", cachedVoltage);
    }

    /**
     * Sets the target velocity in RPM.
     *
     * @param rpm Revolutions Per Minute
     */
    public void setTargetVelocity(double rpm) {
        // Convert the user's RPM request to the Hardware's TPS requirement
        this.currentTargetVelocityTPS = rpmToTicks(rpm);
    }

    public double getError() {
        return ticksToRPM(currentTargetVelocityTPS - currentVelocityTPS);
    }

    public double getTargetVelocity() {
        return ticksToRPM(currentTargetVelocityTPS);
    }
    // --- Helper Conversion Methods ---

    private double rpmToTicks(double rpm) {
        // RPM / 60 = RPS
        // RPS * CPR = TPS
        return (rpm / 60.0) * (MOTOR_CPR * GEAR_RATIO);
    }

    private double ticksToRPM(double tps) {
        // TPS / CPR = RPS
        // RPS * 60 = RPM
        return (tps / (MOTOR_CPR * GEAR_RATIO)) * 60.0;
    }
}