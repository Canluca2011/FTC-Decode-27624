package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * A simple command that runs the shooter at a specific velocity while active,
 * and stops the shooter when it ends.
 */
public class RunIntakeCommand extends CommandBase {

    private final Intake intake;

    public RunIntakeCommand(Intake intake) {
        this.intake = intake;
        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(1.0);
    }

    @Override
    public void execute() {
        intake.setSpeed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released)
        intake.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs until it is explicitly interrupted
        return false;
    }
}