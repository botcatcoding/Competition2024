package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;

public class CenterNoteConditional extends Command {
    public static boolean intaking;

    public CenterNoteConditional() {

    }

    @Override
    public void initialize() {
        intaking = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intaking = false;
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
