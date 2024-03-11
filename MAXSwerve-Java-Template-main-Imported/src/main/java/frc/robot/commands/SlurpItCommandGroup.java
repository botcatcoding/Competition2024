package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slurp;

public class SlurpItCommandGroup extends SequentialCommandGroup {
    public SlurpItCommandGroup(Slurp sl, DigitalInput sd, Shooter sh) {
        SlurpCommand scFRC = new SlurpCommand(-6000, true, true, sl, sd);
        SlurpCommand scSND = new SlurpCommand(-6000, true, false, sl, sd);
        WaitCommand timer = new WaitCommand(.25);
        SlurpCommand scTRD = new SlurpCommand(0000, true, true, sl, sd);
        ShootCommand sht = new ShootCommand(sh, -10, true, false);

        // Command SlurpIt =
        // scFRC.andThen(scSND).raceWith(timer).andThen(scTRD).alongWith(sht);
        Command waiter = timer.deadlineWith(scSND);
        Command slurpBack = scTRD.deadlineWith(sht);
        addCommands(scFRC, waiter, slurpBack);
    }
}
