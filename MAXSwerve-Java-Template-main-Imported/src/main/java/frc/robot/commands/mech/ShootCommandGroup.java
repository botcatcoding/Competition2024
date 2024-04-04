package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slurp;

public class ShootCommandGroup extends SequentialCommandGroup {
    public ShootCommandGroup(Shooter sh, Slurp sl, DigitalInput sd, Kinamatics k, boolean ss) {
        ShootCommandWithLength spinUp = new ShootCommandWithLength(sh, k, true);
        ShootCommandWithLength keepUp = new ShootCommandWithLength(sh, k, true);
        WaitCommand sleep = new WaitCommand(1);
        SlurpCommand dontFeedIt = new SlurpCommand(0, true, false, sl, sd);
        SlurpCommand feedIt = new SlurpCommand(-6000, true, false, sl, sd);

        ParallelCommandGroup shootIt = new ParallelCommandGroup(feedIt, keepUp);
        ParallelRaceGroup spinUpButDontFeedIt = new ParallelRaceGroup(dontFeedIt, spinUp);
        if (ss) {
            addCommands(spinUpButDontFeedIt, sleep.deadlineWith(shootIt));
        } else {
            addCommands(spinUpButDontFeedIt, shootIt);
        }

    }
}
