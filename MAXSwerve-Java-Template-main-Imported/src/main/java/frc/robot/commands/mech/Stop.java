package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class Stop extends Command {
    boolean brakeMode;
    Elbow elbow;
    Shoulder shoulder;

    public Stop(Elbow e, Shoulder sh, boolean shouldBreak) {
        elbow = e;
        shoulder = sh;
        brakeMode = shouldBreak;

    }

    @Override
    public void initialize() {
        elbow.setBrake(brakeMode);
        shoulder.setBrake(brakeMode);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
