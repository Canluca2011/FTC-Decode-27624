package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.subsystems.Led.RobotState.RGB_CYCLE;
import static org.firstinspires.ftc.teamcode.subsystems.Led.RobotState.SHOOTER_IDLE;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commands.RunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.RunShooterDistanceCommand;
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

import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode.Alliance;

/**
 * LongCycle
 * Same opening as Long9 (preload shot + one intake cycle = first two shots), then instead of the
 * rest it shuttles to the (9, 11) @ 180 deg pickup point and back to the shoot spot three times,
 * shooting after each return. Total = 5 shots. Extend and pass the Alliance.
 */
public abstract class LongCycle extends CommandOpMode {

    protected final Alliance alliance;

    private Follower follower;
    private TelemetryData telemetryData;

    // Subsystems
    private Shooter mShooter;
    private Intake mIntake;
    private Feeder mFeeder;
    private Vision mVision;
    private Drive mDrive;
    private Led mLed;
    private PrismSubsystem mPrismSubsystem;

    // Opening paths (first two shots) + the reusable shuttle legs + the park.
    private PathChain Path1, Path2, Path3, PathOut, PathBack, PathPark;

    public LongCycle(Alliance alliance) {
        this.alliance = alliance;
    }

    // ---------------------------------------------------------------------------------------------
    // MIRRORING HELPERS
    // ---------------------------------------------------------------------------------------------

    public Pose pose(double x, double y, double headingDeg) {
        if (alliance == Alliance.RED) {
            double mirroredX = 144.0 - x;
            double mirroredHeading = 180.0 - headingDeg;
            return new Pose(mirroredX, y, Math.toRadians(mirroredHeading));
        }
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    public Pose point(double x, double y) {
        if (alliance == Alliance.RED) {
            return new Pose(144.0 - x, y);
        }
        return new Pose(x, y);
    }

    public double radians(double degrees) {
        if (alliance == Alliance.RED) {
            return Math.toRadians(180.0 - degrees);
        }
        return Math.toRadians(degrees);
    }

    public double degrees(double degrees) {
        if (alliance == Alliance.RED) {
            return 180.0 - degrees;
        }
        return degrees;
    }

    // ---------------------------------------------------------------------------------------------
    // PATH BUILDING
    // ---------------------------------------------------------------------------------------------
    public void buildPaths() {

        // Path1: Start (57, 9) -> Shoot Position (57, 15)
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(57.000, 9.000, 90),
                                pose(57.000, 15.000, 112)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(90),
                        radians(112)
                )
                .build();

        // Path2: Shoot Position -> Intake Sample 1 (9, 35.5)
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                pose(57.000, 15.000, 112),
                                point(59.601, 37.094),
                                pose(9.000, 35.500, 180)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(112),
                        radians(180)
                )
                .build();

        // Path3: Intake Sample 1 -> Shoot Position (57, 21)
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(9.000, 35.500, 180),
                                pose(57.000, 21.000, 114)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(180),
                        radians(114)
                )
                .build();

        // PathOut: Shoot Position (57, 21) -> Pickup point (9, 11) @ 180. Reused every cycle.
        PathOut = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(57.000, 21.000, 114),
                                pose(9.000, 11.000, 180)
                        )
                )
                // Finish the turn to 180 by the halfway point so we're facing the pickup before the
                // translation ends (third arg = fraction of the path over which heading completes).
                .setLinearHeadingInterpolation(
                        radians(114),
                        radians(180),
                        0.5
                )
                .build();

        // PathBack: Pickup point (9, 11) -> Shoot Position (57, 21). Reused every cycle.
        PathBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(9.000, 11.000, 180),
                                pose(57.000, 21.000, 114)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(180),
                        radians(114)
                )
                .build();

        // PathPark: Shoot Position (57, 21) -> Park (32, 40) @ 180. Same park as Long9 (Path6).
        PathPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(57.000, 21.000, 114),
                                pose(51.000, 25.000, 120)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(114),
                        radians(120)
                )
                .build();
    }

    // ---------------------------------------------------------------------------------------------
    // INITIALIZATION & RUN
    // ---------------------------------------------------------------------------------------------

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

        CommandScheduler.getInstance().reset();
        telemetryData = new TelemetryData(telemetry);

        mShooter = new Shooter(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap);
        mFeeder = new Feeder(hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mDrive = new Drive(follower);
        mLed = new Led(hardwareMap);
        mPrismSubsystem = new PrismSubsystem(hardwareMap, telemetryData);

        register(mShooter, mIntake, mFeeder, mVision, mDrive, mLed, mPrismSubsystem);
        PanelsConfigurables.INSTANCE.refreshClass(PrismConfig.INSTANCE);

        buildPaths();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Initialize Follower with the start pose of Path 1
        follower.setStartingPose(pose(57.000, 9.000, 90));
        mLed.setState(RGB_CYCLE);

        schedule(
                new RunCommand(() -> { mDrive.updateLocalization(); follower.update(); }),
                new SequentialCommandGroup(
                        // Path 1: Move to first shoot position + Spin up shooter
                        new FollowPathCommand(follower, Path1).alongWith(new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                        // Shoot 1
                        alignAndShoot().withTimeout(2250),

                        // Path 2: Go to Intake (Race with intake running)
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path2)),

                        // Path 3: Return to Shoot + Spin up
                        new FollowPathCommand(follower, Path3).alongWith(new RunIntakeCommand(mIntake).withTimeout(750), new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                        // Shoot 2
                        alignAndShoot().withTimeout(2250),

                        // Shuttle to (9, 11) and back, shooting after each return. 3 times.
                        shuttleCycle(),
                        shuttleCycle(),
                        shuttleCycle(),

                        // Park (same end pose as Long9)
                        new FollowPathCommand(follower, PathPark)
                )
        );
    }

    /**
     * One shuttle cycle: drive out to the (9, 11) pickup with the intake running, drive back to the
     * shoot spot while spinning up, then align and shoot. A fresh instance is built per call so the
     * same command objects are never scheduled more than once.
     */
    private Command shuttleCycle() {
        return new SequentialCommandGroup(
                // Out to pickup (9, 11) @ 180 with intake running
                new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, PathOut)),

                // Back to shoot position + spin up
                new FollowPathCommand(follower, PathBack).alongWith(new RunIntakeCommand(mIntake).withTimeout(750), new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                // Shoot
                alignAndShoot().withTimeout(1750)
        );
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

        super.run();
        PoseStorage.currentPose = follower.getPose();
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        // --- Pinpoint health ---
        telemetryData.addData("Pinpoint", mDrive.getLocalizationStatus());
        telemetryData.addData("PP Healthy", mDrive.isLocalizationHealthy());
        telemetryData.addData("PP Faults", mDrive.getLocalizationFaultCount());
        telemetryData.addData("PP Resyncs", mDrive.getLocalizationResyncCount());
        telemetryData.addData("PP Freq(Hz)", mDrive.getPinpointFrequency());
        telemetryData.update();
    }

    public Command alignAndShoot() {
        return new AlignToTagCommand(
                mVision,
                mDrive,
                () -> 0.0,
                () -> 0.0,
                degrees(114.0),
                alliance
        ).alongWith(new RunShooterDistanceCommand(mShooter, mVision), new ShootFeedCommand(mFeeder, mIntake, mShooter, mLed, mDrive::isAligned));
    }
}
