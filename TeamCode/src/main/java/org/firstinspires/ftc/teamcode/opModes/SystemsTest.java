package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.PrismConfig;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "SystemsTest", group = "Test")
public class SystemsTest extends CommandOpMode {

    // ===== Tunable Constants =====
    private static final double TARGET_SHAFT_RPM = 5000;
    private static final long RPM_TIMEOUT_MS = 5000;
    private static final double INDICATOR_RED_POS = 0.28;
    private static final double INDICATOR_GREEN_POS = 0.48;
    private static final double INDICATOR_ON_POS = 0.48;

    // Subsystems
    private Shooter mShooter;
    private Intake mIntake;
    private Feeder mFeeder;
    private Vision mVision;
    private Led mLed;
    private PrismSubsystem mPrismSubsystem;
    private TelemetryData telemetryData;

    // Phase 3 tracking
    private boolean phase3TagVisible = false;

    @Override
    public void initialize() {
        telemetryData = new TelemetryData(telemetry);

        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mLed = new Led(hardwareMap);
        mPrismSubsystem = new PrismSubsystem(hardwareMap, telemetryData);

        register(mShooter, mIntake, mFeeder, mVision, mLed, mPrismSubsystem);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        schedule(
                new SequentialCommandGroup(
                        phase0(),
                        phase1(),
                        phase2(),
                        phase3()
                )
        );
    }

    @Override
    public void run() {
        super.run();
        telemetryData.update();
    }

    // =======================================================================
    // Phase 0 — Lights On (rainbow + indicator ON) + warning, ~1 s hold
    // =======================================================================
    private Command phase0() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addData("Phase", "0 — Lights On + Warn");
                    telemetry.addData("WARNING", "Spinning up at full power — keep clear, NO ball loaded.");
                    configAllStripsRainbow();
                    PrismConfig.Indicator1_Mode = PrismConfig.IndicatorMode.STATIC;
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.GREEN;
                }),
                new WaitCommand(1000)
        );
    }

    // =======================================================================
    // Phase 1 — Motor RPM Test (full power, raw shaft RPM, NO PID)
    // =======================================================================
    private Command phase1() {
        return new RpmTestCommand();
    }

    // =======================================================================
    // Phase 2 — Flash Twice (motors OK signal)
    // =======================================================================
    private Command phase2() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> telemetry.addData("Phase", "2 — Flash Twice (Motors OK)")),
                new InstantCommand(() -> {
                    configAllStripsSolid(255, 255, 255);
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.GREEN;
                }),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    configAllStripsOff();
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.BLACK;
                }),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    configAllStripsSolid(255, 255, 255);
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.GREEN;
                }),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    configAllStripsOff();
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.BLACK;
                }),
                new WaitCommand(200)
        );
    }

    // =======================================================================
    // Phase 3 — Limelight Detection (continuous until OpMode stop)
    // =======================================================================
    private Command phase3() {
        return new SequentialCommandGroup(
                // Set up wildcard mode + neutral state once on entry
                new InstantCommand(() -> {
                    mVision.setTargetTagId(-1);
                    configAllStripsRainbow();
                    PrismConfig.Indicator1_Mode = PrismConfig.IndicatorMode.STATIC;
                    PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.BLUE;
                    phase3TagVisible = false;
                    telemetry.addData("Phase", "3 — Limelight Detection (any AprilTag)");
                }),
                // Track tag visibility live, switch lights accordingly
                new RunCommand(() -> {
                    boolean visible = mVision.isTargetVisibleNow();

                    if (visible != phase3TagVisible) {
                        phase3TagVisible = visible;
                        if (visible) {
                            configAllStripsSolid(0, 255, 0);
                            PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.GREEN;
                        } else {
                            configAllStripsRainbow();
                            PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.BLUE;
                        }
                    }

                    telemetry.addData("Phase", "3 — Limelight Detection");
                    telemetry.addData("Target Visible Now", visible);
                    telemetry.addData("Has Valid Target", mVision.hasValidTarget());
                    telemetry.addData("tx", mVision.getSteeringError());
                    telemetry.addData("Distance (m)", mVision.getDistance());
                })
        );
    }

    // =======================================================================
    // Custom Command — RPM test with pass/fail + intake/feeder pulse
    // =======================================================================
    private class RpmTestCommand extends CommandBase {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean passed = false;
        private boolean intakePulsed = false;

        public RpmTestCommand() {
            addRequirements(mShooter, mIntake, mFeeder);
        }

        @Override
        public void initialize() {
            mShooter.setTestFullPower(true);
            timer.reset();
            passed = false;
            intakePulsed = false;
            telemetry.addData("Phase", "1 — RPM Test (full power)");
        }

        @Override
        public void execute() {
            double rpmL = mShooter.getRawShaftRpmL();
            double rpmR = mShooter.getRawShaftRpmR();
            double elapsed = timer.milliseconds();

            // Pulse intake + feeder briefly at ~1 s as a visual check
            if (elapsed > 1000 && !intakePulsed) {
                intakePulsed = true;
                mIntake.setSpeed(1.0);
                mFeeder.setSpeed(1.0);
            }
            if (intakePulsed && elapsed > 2000) {
                mIntake.setSpeed(0.0);
                mFeeder.setSpeed(0.0);
            }

            telemetry.addData("Phase 1 RPM L", rpmL);
            telemetry.addData("Phase 1 RPM R", rpmR);
            telemetry.addData("Phase 1 Target", TARGET_SHAFT_RPM);
            telemetry.addData("Phase 1 Elapsed (ms)", (long) elapsed);

            if (rpmL >= TARGET_SHAFT_RPM && rpmR >= TARGET_SHAFT_RPM) {
                // PASS — both shafts reached target
                passed = true;
                mShooter.setTestFullPower(false);
                mIntake.setSpeed(0.0);
                mFeeder.setSpeed(0.0);
                telemetry.addData("Phase 1 Result", "PASS");
                telemetry.addData("Phase 1", "Complete — stopping motors");
            } else if (elapsed >= (double) RPM_TIMEOUT_MS) {
                // FAIL — timeout without reaching target
                mShooter.setTestFullPower(false);
                mIntake.setSpeed(0.0);
                mFeeder.setSpeed(0.0);

                configAllStripsSolid(255, 0, 0);
                PrismConfig.Indicator1_Mode = PrismConfig.IndicatorMode.STATIC;
                PrismConfig.Indicator1_StaticColor = PrismConfig.IndicatorStaticColor.RED;

                telemetry.addData("Phase 1 Result", "FAIL");
                telemetry.addData("FAIL RPM L", rpmL);
                telemetry.addData("FAIL RPM R", rpmR);
                if (rpmL < TARGET_SHAFT_RPM && rpmR < TARGET_SHAFT_RPM) {
                    telemetry.addData("FAIL Reason", "Both shafts below " + TARGET_SHAFT_RPM);
                } else if (rpmL < TARGET_SHAFT_RPM) {
                    telemetry.addData("FAIL Reason", "Left shaft (" + (int) rpmL + " RPM) below " + TARGET_SHAFT_RPM);
                } else {
                    telemetry.addData("FAIL Reason", "Right shaft (" + (int) rpmR + " RPM) below " + TARGET_SHAFT_RPM);
                }
            }
        }

        @Override
        public boolean isFinished() {
            return passed;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                mShooter.setTestFullPower(false);
                mIntake.setSpeed(0.0);
                mFeeder.setSpeed(0.0);
            }
        }
    }

    // =======================================================================
    // PrismConfig helpers
    // =======================================================================
    private static void configAllStripsRainbow() {
        PrismConfig.Strip0_Anim = PrismConfig.StripAnim.RAINBOW;
        PrismConfig.Strip0_Brightness = 100;
        PrismConfig.Strip0_Speed = 0.5f;
        PrismConfig.Strip0_Direction = PrismConfig.Direction.FORWARD;

        PrismConfig.Strip1_Anim = PrismConfig.StripAnim.RAINBOW;
        PrismConfig.Strip1_Brightness = 100;
        PrismConfig.Strip1_Speed = 0.5f;
        PrismConfig.Strip1_Direction = PrismConfig.Direction.FORWARD;

        PrismConfig.Strip2_Anim = PrismConfig.StripAnim.RAINBOW;
        PrismConfig.Strip2_Brightness = 100;
        PrismConfig.Strip2_Speed = 0.5f;
        PrismConfig.Strip2_Direction = PrismConfig.Direction.FORWARD;

        PrismConfig.Strip3_Anim = PrismConfig.StripAnim.RAINBOW;
        PrismConfig.Strip3_Brightness = 100;
        PrismConfig.Strip3_Speed = 0.5f;
        PrismConfig.Strip3_Direction = PrismConfig.Direction.FORWARD;
    }

    private static void configAllStripsSolid(int r, int g, int b) {
        PrismConfig.Strip0_Anim = PrismConfig.StripAnim.SOLID;
        PrismConfig.Strip0_Color1_R = r;
        PrismConfig.Strip0_Color1_G = g;
        PrismConfig.Strip0_Color1_B = b;
        PrismConfig.Strip0_Brightness = 100;

        PrismConfig.Strip1_Anim = PrismConfig.StripAnim.SOLID;
        PrismConfig.Strip1_Color1_R = r;
        PrismConfig.Strip1_Color1_G = g;
        PrismConfig.Strip1_Color1_B = b;
        PrismConfig.Strip1_Brightness = 100;

        PrismConfig.Strip2_Anim = PrismConfig.StripAnim.SOLID;
        PrismConfig.Strip2_Color1_R = r;
        PrismConfig.Strip2_Color1_G = g;
        PrismConfig.Strip2_Color1_B = b;
        PrismConfig.Strip2_Brightness = 100;

        PrismConfig.Strip3_Anim = PrismConfig.StripAnim.SOLID;
        PrismConfig.Strip3_Color1_R = r;
        PrismConfig.Strip3_Color1_G = g;
        PrismConfig.Strip3_Color1_B = b;
        PrismConfig.Strip3_Brightness = 100;
    }

    private static void configAllStripsOff() {
        configAllStripsSolid(0, 0, 0);
    }
}
