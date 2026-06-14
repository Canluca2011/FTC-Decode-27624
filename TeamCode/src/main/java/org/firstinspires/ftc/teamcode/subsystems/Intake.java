package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

public class Intake extends SubsystemBase {
    private DcMotorSimple intake;
    private double intakeSpeed = 0.0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
    }

    @Override
    public void periodic() {
        intake.setPower(-intakeSpeed);
    }

    public void setSpeed(double speed) {
        intakeSpeed = speed;
    }
}
