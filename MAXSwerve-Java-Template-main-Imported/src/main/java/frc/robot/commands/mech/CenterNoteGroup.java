package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slurp;

//CENTERING STUFF
public class CenterNoteGroup extends SequentialCommandGroup {
    public CenterNoteGroup(Lighting lighting, Slurp sl, DigitalInput sd, Shooter sh) {
        Command gotNoteCommand = Lighting.constructNotificationCommand(lighting, "G");
        SlurpCommand scSND = new SlurpCommand(-6000, true, false, sl, sd);
        WaitCommand timer = new WaitCommand(.5);
        SlurpCommand scTRD = new SlurpCommand(1000, true, true, sl, sd);
        ShootCommand sht = new ShootCommand(sh, -10, true, false);
        Command stopShootAlso = new ShootCommand(sh, 0, true, false);

        Command waiter = timer.deadlineWith(scSND, stopShootAlso);
        Command slurpBack = scTRD.deadlineWith(sht).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        addCommands(gotNoteCommand, waiter, slurpBack);
    }
}
