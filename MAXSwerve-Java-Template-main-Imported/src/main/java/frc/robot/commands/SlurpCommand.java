package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Slurp;

public class SlurpCommand extends Command {
    double speed = 0;
    Slurp slurp;

    public SlurpCommand(double s, Slurp sl) {
        addRequirements(sl);
        speed = s;
        slurp = sl;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // feed.setFeed(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
