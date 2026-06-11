package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

public class Feeder extends SubsystemBase {
    private DcMotorSimple feeder;
    private double feedSpeed = 0.0;

    public Feeder(HardwareMap hardwareMap) {
        feeder = hardwareMap.get(DcMotorSimple.class, "index");
    }

    @Override
    public void periodic() {
        feeder.setPower(-feedSpeed);
    }

    public void setSpeed(double speed) {
        feedSpeed = speed;
    }
}
