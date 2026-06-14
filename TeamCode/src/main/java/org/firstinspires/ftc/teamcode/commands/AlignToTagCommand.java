package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.opModes.BaseShooterOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AlignToTagCommand extends CommandBase {

    private final Vision vision;
    private final Drive drive;
    private final DoubleSupplier xAxis, yAxis;
    private final double targetHeadingRadians;

    // --- Tuning Constants ---
    // PD steers toward the tag. kStatic is a feedforward that guarantees just enough power to break
    // static friction for the final degrees, so we can run a LOW Kp (no overshoot) and still reach
    // the target instead of stalling short of it.
    private static final double Kp = 0.012;      // lowered from 0.022 to kill the overshoot
    private static final double Kd = 0.01;     // more damping for momentum + vision latency
    private static final double kStatic = 0.01;  // min turn power to overcome friction near target

    // Inside this error band (tx units) we count as aligned and stop turning, so the robot settles
    // instead of hunting back and forth across the target.
    private static final double TURN_TOLERANCE = 2.65;

    private static final double MAX_VISION_SPEED = 0.5;

    private static final double SCAN_SPEED = 0.3;

    // State Variables
    private double previousTx = 0;
    private boolean hasPreviousTx = false;

    // 0 = Not scanning (or undecided), 1 = Left, -1 = Right
    private double lockedScanDirection = 0;
    private BaseShooterOpMode.Alliance alliance;

    public AlignToTagCommand(Vision vision, Drive drive, DoubleSupplier xAxis, DoubleSupplier yAxis, double targetHeadingDegrees, BaseShooterOpMode.Alliance alliance) {
        this.vision = vision;
        this.drive = drive;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        this.alliance = alliance;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        previousTx = 0;
        hasPreviousTx = false;
        lockedScanDirection = 0; // Reset on start
        drive.aligned = false;

        drive.startTeleopDrive();
    }

    @Override
    public void execute() {
        double turnPower;

        if (vision.hasValidTarget()) {
            // ===========================================
            // TARGET FOUND: UNLOCK AND TRACK
            // ===========================================

            // We see the tag, so we don't need our "blind guess" anymore.
            // Reset this to 0 so next time we lose the tag, we recalculate fresh.
            lockedScanDirection = 0;

            double tx = vision.getSteeringError();

            if(vision.getDistance() > 2.65) {
                tx += alliance == BaseShooterOpMode.Alliance.BLUE ? -2.0 : 2.0;
            }

            // Guard the first frame after (re)acquiring the tag so previousTx=0 doesn't cause a
            // huge fake derivative kick.
            double derivative = hasPreviousTx ? (tx - previousTx) : 0.0;
            previousTx = tx;
            hasPreviousTx = true;

            boolean aligned = Math.abs(tx) <= TURN_TOLERANCE;
            drive.aligned = aligned;

            if (aligned) {
                // Within the error margin: hold so we settle instead of hunting.
                turnPower = 0.0;
            } else {
                double pd = -(Kp * tx + Kd * derivative);
                // Static feedforward in the direction we're already turning: guarantees the last
                // few degrees still move under a low Kp instead of stalling short of the target.
                pd += Math.copySign(kStatic, pd);
                turnPower = Math.max(-MAX_VISION_SPEED, Math.min(MAX_VISION_SPEED, pd));
            }

        } else {
            // ===========================================
            // TARGET LOST: COMMIT TO A DIRECTION
            // ===========================================

            // Only calculate the direction IF we haven't picked one yet.
            if (lockedScanDirection == 0) {
                double currentHeading = drive.getFollower().getPose().getHeading();
                double angleDifference = getShortestAngleDiff(targetHeadingRadians, currentHeading);

                // Determine direction (-1 or 1)
                // If angleDifference is 0 (extremely rare), default to 1
                lockedScanDirection = (angleDifference >= 0) ? 1.0 : -1.0;
            }

            // Just apply the locked speed.
            // We ignore the current heading now. We trust the turn.
            turnPower = lockedScanDirection * SCAN_SPEED;

            previousTx = 0;
            hasPreviousTx = false;
            drive.aligned = false;
        }

        drive.joystickDrive(xAxis.getAsDouble(), yAxis.getAsDouble(), turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        drive.joystickDrive(0, 0, 0);
        drive.aligned = false;
    }

    private double getShortestAngleDiff(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}