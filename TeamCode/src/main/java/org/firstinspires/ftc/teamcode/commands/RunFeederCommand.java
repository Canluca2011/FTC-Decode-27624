package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * A simple command that runs the shooter at a specific velocity while active,
 * and stops the shooter when it ends.
 */
public class RunFeederCommand extends CommandBase {

    private final Feeder feeder;

    public RunFeederCommand(Feeder feeder) {
        this.feeder = feeder;
        // Declare subsystem dependency so the scheduler knows
        // this command requires the Shooter subsystem.
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(-1.0);
    }

    @Override
    public void execute() {
        feeder.setSpeed(-1.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (button released)
        feeder.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Return false so the command runs until it is explicitly interrupted
        return false;
    }
}