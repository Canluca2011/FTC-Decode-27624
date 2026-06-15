package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.Led.RobotState.RGB_CYCLE;
import static org.firstinspires.ftc.teamcode.subsystems.Led.RobotState.SHOOTER_IDLE;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commands.EmergencyCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterDistanceCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootFeedCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.PrismConfig;
import org.firstinspires.ftc.teamcode.subsystems.PrismSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * BaseShooterOpMode
 * Refactored to use a Constructor for Alliance configuration.
 */
public abstract class BaseShooterOpMode extends CommandOpMode {

    // Define the Enum directly here or in a separate file
    public enum Alliance {
        RED,
        BLUE
    }

    // This field stores which alliance we are running
    protected final Alliance alliance;

    // Constructor required by Child classes
    public BaseShooterOpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    protected TelemetryData telemetryData;
    protected Follower follower;
    protected Shooter mShooter;
    protected Intake mIntake;
    protected Feeder mFeeder;
    protected Vision mVision;
    protected Drive mDrive;
    protected Led mLed;
    protected PrismSubsystem mPrismSubsystem;
    protected GamepadEx controller;
    protected int shouldInvertX, shouldInvertY;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        // 1. RESET FIRST
        CommandScheduler.getInstance().reset();

        telemetryData = new TelemetryData(telemetry);

        // 2. Initialize Subsystem
        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mDrive = new Drive(follower);
        mLed = new Led(hardwareMap);
        mPrismSubsystem = new PrismSubsystem(hardwareMap, telemetryData);

        // LOGIC: Determine Tag ID based on the alliance variable
        int targetTagId = (alliance == Alliance.BLUE) ? 20 : 24;
        mVision.setTargetTagId(targetTagId);

        shouldInvertX = alliance == Alliance.BLUE ? -1 : 1;
        shouldInvertY = alliance == Alliance.BLUE ? -1 : 1;

        // 3. Register

        register(mShooter, mIntake, mFeeder, mVision, mDrive, mLed, mPrismSubsystem);
        PanelsConfigurables.INSTANCE.refreshClass(PrismConfig.INSTANCE);

        // 4. Initialize Controller
        controller = new GamepadEx(gamepad1);

        // 5. Bind Buttons
        bindButtons();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Carry the heading across auto -> teleop from the stored snapshot. The Pinpoint restarts its
        // tracking from 0 across the OpMode boundary, so we cannot read the heading live here - the
        // last value auto saved into PoseStorage is the source of truth.
        follower.setPose(new Pose(0.0, 0.0, PoseStorage.currentPose.getHeading()));
        follower.startTeleOpDrive();

        mLed.setState(RGB_CYCLE);
    }

    @Override
    public void initialize_loop() {
        mLed.periodic();
    }

    @Override
    public void run() {
        if(mLed.getState() == RGB_CYCLE) {
            mLed.setState(SHOOTER_IDLE);
        }
        // 1. CRITICAL: Read Controller Inputs
        controller.readButtons();

        // 2. Run Scheduler
        super.run();
        // Guard localization BEFORE updating the follower so a Pinpoint glitch/reboot is detected
        // and re-synced this same loop instead of corrupting the field-oriented heading.
        mDrive.updateLocalization();
        follower.update();
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        // --- Pinpoint health (watch these to see if the odometry computer hiccupped) ---
        telemetryData.addData("Pinpoint", mDrive.getLocalizationStatus());
        telemetryData.addData("PP Healthy", mDrive.isLocalizationHealthy());
        telemetryData.addData("PP Faults", mDrive.getLocalizationFaultCount());
        telemetryData.addData("PP Resyncs", mDrive.getLocalizationResyncCount());
        telemetryData.addData("PP Freq(Hz)", mDrive.getPinpointFrequency());
        // 3. Update Telemetry
        telemetryData.update();
    }

    private void bindButtons() {
        mDrive.setDefaultCommand(new TeleopDriveCommand(mDrive, () -> -gamepad1.left_stick_y * shouldInvertY, () -> -gamepad1.left_stick_x * shouldInvertX, () -> -gamepad1.right_stick_x));

        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new AlignToTagCommand(
                        mVision,
                        mDrive,
                        () -> -gamepad1.left_stick_y * shouldInvertY * 0.3,
                        () -> -gamepad1.left_stick_x * shouldInvertX * 0.3,
                        (alliance == Alliance.BLUE) ? 130.0 : 50.0, alliance
                ).alongWith(new RunShooterDistanceCommand(mShooter, mVision), new ShootFeedCommand(mFeeder, mIntake, mShooter, mLed, mDrive::isAligned)));

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenHeld(new EmergencyCommand(mFeeder,mIntake,mShooter));

        controller.getGamepadButton(GamepadKeys.Button.START)
                .whenHeld(new InstantCommand(() -> mDrive.resetHeading()));

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new RunIntakeCommand(mIntake));
    }
}