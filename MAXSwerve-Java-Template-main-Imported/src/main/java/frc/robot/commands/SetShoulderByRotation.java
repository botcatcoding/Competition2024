package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class SetShoulderByRotation extends Command {
    Shoulder shoulder;
    double position;
    boolean shouldEnd;

    public SetShoulderByRotation(double pos, boolean se, Shoulder sh) {
        addRequirements(sh);
        shoulder = sh;
        position = pos;
        shouldEnd = se;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shoulder.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.stop();
    }

    @Override
    public boolean isFinished() {
        return shoulder.isPostioned(position) && shouldEnd;

    }
}
