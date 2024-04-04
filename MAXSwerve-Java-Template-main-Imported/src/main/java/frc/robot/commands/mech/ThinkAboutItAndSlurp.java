package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slurp;

public class ThinkAboutItAndSlurp extends Command {
    CenterNoteGroup sitc;

    public ThinkAboutItAndSlurp(Lighting lighting, Slurp slurp, DigitalInput slurpDetect, Shooter shooter) {
        sitc = new CenterNoteGroup(lighting, slurp, slurpDetect, shooter);
    }

    @Override
    public void initialize() {
        if (CenterNoteConditional.intaking) {
            sitc.schedule();
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
