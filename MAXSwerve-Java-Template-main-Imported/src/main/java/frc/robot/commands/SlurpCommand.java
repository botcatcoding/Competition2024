package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Slurp;

public class SlurpCommand extends Command {
    double speed = 0;
    Slurp slurp;
    DigitalInput slurpDetect;
    boolean velocityMode;
    boolean stopIfLimitSwitch;

    public SlurpCommand(double s, boolean vm, boolean ss, Slurp sl, DigitalInput sd) {
        addRequirements(sl);
        speed = s;
        slurp = sl;
        slurpDetect = sd;
        velocityMode = vm;
        stopIfLimitSwitch = ss;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("switch", slurpDetect.get());
        if (velocityMode) {
            slurp.setFeedVelo(speed);

        } else {
            slurp.setFeedPercent(speed);
        }

    }

    @Override
    public void end(boolean interrupted) {
        slurp.setFeedPercent(0);
    }

    @Override
    public boolean isFinished() {
        return slurpDetect.get() && stopIfLimitSwitch;

    }
}
