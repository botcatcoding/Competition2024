package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;

public class SetElbowByRotation extends Command {

    Elbow elbow;
    double position;
    boolean shouldEnd;
    double deadband;

    public SetElbowByRotation(double pos, boolean se, double dead, Elbow e) {
        addRequirements(e);
        elbow = e;
        position = pos;
        shouldEnd = se;
        deadband = dead;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elbow.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        // elbow.stop();
        // elbow.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return elbow.isPostioned(position, deadband) && shouldEnd;
    }
}
