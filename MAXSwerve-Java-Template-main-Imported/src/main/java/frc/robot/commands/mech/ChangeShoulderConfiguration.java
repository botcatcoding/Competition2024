package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ChangeShoulderConfiguration extends Command {

    Shoulder s;
    boolean fast;

    public ChangeShoulderConfiguration(Shoulder shoulder, boolean fast, boolean afterWait) {
        addRequirements(shoulder);
        s = shoulder;
        this.fast = fast;
    }

    @Override
    public void initialize() {
        if (fast) {
            s.applyConfigurationRegular();
        } else {
            s.applyConfigurationClimbSlow();
        }
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
