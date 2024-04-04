package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elbow;

public class ChangeElbowConfiguration extends Command {

    Elbow e;
    boolean fast;

    public ChangeElbowConfiguration(Elbow elbow, boolean fast, boolean afterWait) {
        addRequirements(elbow);
        e = elbow;
        this.fast = fast;
    }

    @Override
    public void initialize() {
        if (fast) {
            e.applyConfigurationRegular();
        } else {
            e.applyConfigurationClimbSlow();
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
