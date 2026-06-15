package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import java.util.List;

public class Vision extends SubsystemBase {

    private final Limelight3A limelight;
    private final TelemetryData telemetry;

    // --- Configuration Constants ---
    private static final double CAMERA_LENS_HEIGHT = 0.365;
    private static final double TARGET_HEIGHT = 29.25 * 0.0254;
    private static final double CAMERA_MOUNT_ANGLE_DEGREES = 30.5;

    // Occlusion Handling:
    // If a ball blocks the camera, we keep the last 'tx' and 'distance' for this many ms.
    private static final long TARGET_MEMORY_MS = 250;

    // --- State Variables ---
    private double currentDistance = 0.0;
    private double currentTx = 0.0;
    private double currentTy = 0.0;
    private double currentTa = 0.0;

    private boolean isCurrentlyVisible = false;
    private long lastValidTargetTime = 0;
    private int targetTagId = -1;

    public Vision(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addData("Limelight", "Initialized & Started");
        telemetry.update();
    }

    @Override
    public void periodic() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        boolean validFrameInput = false;
        double rawTx = 0;
        double rawTy = 0;
        double rawTa = 0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (targetTagId != -1) {
                // Strict ID mode
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (tag.getFiducialId() == targetTagId) {
                        rawTx = -tag.getTargetYDegrees();
                        rawTy = tag.getTargetXDegrees();
                        rawTa = tag.getTargetArea();
                        validFrameInput = true;
                        break;
                    }
                }
            } else {
                // Wildcard mode
                rawTx = -result.getTy();
                rawTy = result.getTx();
                rawTa = result.getTa();
                if (Math.abs(rawTx) > 0.001 || Math.abs(rawTy) > 0.001) {
                    validFrameInput = true;
                }
            }
        }

        // --- CORE LOGIC CHANGE ---
        if (validFrameInput) {
            // Case 1: We see the target RIGHT NOW.
            isCurrentlyVisible = true;
            lastValidTargetTime = System.currentTimeMillis();

            // Calculate Distance
            double angleToGoalDegrees = CAMERA_MOUNT_ANGLE_DEGREES + rawTy;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            currentDistance = (TARGET_HEIGHT - CAMERA_LENS_HEIGHT) / Math.tan(angleToGoalRadians);

            // Update Aiming Data directly
            currentTx = rawTx;
            currentTy = rawTy;
            currentTa = rawTa;

        } else {
            // Case 2: We DO NOT see the target right now.
            isCurrentlyVisible = false;

            // Check how long it has been since we last saw it
            long timeSinceLastSeen = System.currentTimeMillis() - lastValidTargetTime;

            if (timeSinceLastSeen > TARGET_MEMORY_MS) {
                // Case 2a: It's been too long (Target truly lost).
                // Reset everything to 0 to prevent the robot from spinning at the last known speed forever.
                currentTx = 0.0;
                currentTy = 0.0;
                currentDistance = 0.0;
            }
            // Case 2b: It has been LESS than 500ms (Ball blocking camera).
            // Do NOTHING. We keep the old currentTx and currentDistance values.
            // This effectively "freezes" the data during the shot.
        }

        // --- Telemetry ---
        telemetry.addData("Limelight", status.getName());
        if (hasValidTarget()) {
            telemetry.addData("Target", isCurrentlyVisible ? "LOCKED" : "CACHED (Occluded)");
            telemetry.addData("tx", currentTx);
            telemetry.addData("Dist", currentDistance);
        } else {
            telemetry.addData("Target", "SEARCHING...");
        }
    }

    // --- Setters & Getters ---

    public void setTargetTagId(int id) {
        this.targetTagId = id;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * WARNING: If the camera is occluded (blocked), this returns the LAST KNOWN value.
     */
    public double getSteeringError() {
        return currentTx;
    }

    public double getDistance() {
        return currentDistance;
    }

    /**
     * @return true if the target is visible OR was visible within the memory threshold.
     */
    public boolean hasValidTarget() {
        // We consider the target "valid" if we see it, OR if we saw it recently.
        return isCurrentlyVisible || (System.currentTimeMillis() - lastValidTargetTime < TARGET_MEMORY_MS);
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * @return true if the target is visible in the current frame (no occlusion/cache).
     */
    public boolean isTargetVisibleNow() {
        return isCurrentlyVisible;
    }

    public void stop() {
        limelight.stop();
    }
}