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
import com.seattlesolvers.solverslib.command.WaitCommand;
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

// Using the Alliance Enum from your BaseShooterOpMode
import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode.Alliance;

/**
 * Short12
 * Contains all auto logic for Blue and Red.
 * Extend this class and pass the Alliance in the constructor to create the OpMode.
 */
public abstract class Long9 extends CommandOpMode {

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

    // Path chains
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6;

    public Long9(Alliance alliance) {
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

        // Path2: Shoot Position -> Intake Sample 1 (16, 35.5)
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

        // Path4: Shoot Position -> Intake Sample 2 (12, 60)
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                pose(57.000, 21.000, 114),
                                point(60.434, 61.476),
                                pose(9.169, 58.975, 180)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(114),
                        radians(180)
                )
                .build();

        // Path5: Intake Sample 2 -> Shoot Position (57, 21)
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(9.169, 58.975, 180),
                                pose(57.000, 21.000, 114)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(180),
                        radians(114)
                )
                .build();

        // Path6: Park / End (32, 40)
        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pose(57.000, 21.000, 114),
                                pose(32.000, 40.000, 180)
                        )
                )
                .setLinearHeadingInterpolation(
                        radians(114),
                        radians(180)
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

        // NOTE: Timings (timeouts/waits) may need adjustment for the new distances
        schedule(
                new RunCommand(() -> { mDrive.updateLocalization(); follower.update(); }),
                new SequentialCommandGroup(
                        // Path 1: Move to first shoot position + Spin up shooter
                        new FollowPathCommand(follower, Path1).alongWith(new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                        // Shoot 1
                        alignAndShoot().withTimeout(3250),

                        // Path 2: Go to Intake (Race with intake running)
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path2)),

                        // Path 3: Return to Shoot + Spin up
                        new FollowPathCommand(follower, Path3).alongWith(new RunIntakeCommand(mIntake).withTimeout(750), new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                        // Shoot 2
                        alignAndShoot().withTimeout(3250),

                        // Path 4: Go to Intake (Race with intake running)
                        new RunIntakeCommand(mIntake).raceWith(new FollowPathCommand(follower, Path4)),

                        // Path 5: Return to Shoot + Spin up
                        new FollowPathCommand(follower, Path5).alongWith(new RunIntakeCommand(mIntake).withTimeout(750), new InstantCommand(() -> mShooter.setTargetVelocity(3000.0))),

                        // Shoot 3
                        alignAndShoot().withTimeout(3250),

                        // Path 6: Park
                        new FollowPathCommand(follower, Path6)
                )
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
        // You may need to adjust the heading target (degrees(130.0)) based on the new shooting position of x=57
        return new AlignToTagCommand(
                mVision,
                mDrive,
                () -> 0.0,
                () -> 0.0,
                degrees(114.0), // Updated to match your path heading at the shooting spot (Path 3/5 end heading)
                alliance
        ).alongWith(new RunShooterDistanceCommand(mShooter, mVision), new ShootFeedCommand(mFeeder, mIntake, mShooter, mLed, mDrive::isAligned));
    }
}