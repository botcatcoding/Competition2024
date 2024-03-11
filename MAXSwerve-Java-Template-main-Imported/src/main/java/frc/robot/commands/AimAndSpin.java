package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class AimAndSpin extends ParallelCommandGroup {
    public AimAndSpin(Shoulder sh, Elbow e, Kinamatics k, Shooter sho) {
        ShootCommand spinUp = new ShootCommand(sho, Constants.MechConstants.shootSpeed, true, false);
        AimAtSpeakerCommand aasc = new AimAtSpeakerCommand(sh, e, k, false);
        addCommands(aasc, spinUp);
    }
}
