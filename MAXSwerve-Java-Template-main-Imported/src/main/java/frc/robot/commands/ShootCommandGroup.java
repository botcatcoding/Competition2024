package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class ShootCommandGroup extends SequentialCommandGroup {
    public ShootCommandGroup(Shooter sh, Slurp sl, DigitalInput sd) {
        ShootCommand spinUp = new ShootCommand(sh, Constants.MechConstants.shootSpeed, true, true);
        ShootCommand keepUp = new ShootCommand(sh, Constants.MechConstants.shootSpeed, true, false);

        SlurpCommand dontFeedIt = new SlurpCommand(0, true, false, sl, sd);
        SlurpCommand feedIt = new SlurpCommand(-6000, true, false, sl, sd);

        ParallelCommandGroup shootIt = new ParallelCommandGroup(feedIt, keepUp);
        ParallelRaceGroup spinUpButDontFeedIt = new ParallelRaceGroup(dontFeedIt, spinUp);

        addCommands(spinUpButDontFeedIt, shootIt);
    }
}
