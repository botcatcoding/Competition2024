package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class TheBigGroup extends SequentialCommandGroup {

    public TheBigGroup(Slurp sl, Shooter sho, Elbow e, Shoulder sh, Kinamatics k, DigitalInput sd) {
        AimAtSpeakerCommand aasc = new AimAtSpeakerCommand(sh, e, k, true);
        ShootCommand spinUp = new ShootCommand(sho, Constants.MechConstants.shootSpeed, true, true);

        ShootCommandGroup scg = new ShootCommandGroup(sho, sl, sd);
        AimAtSpeakerCommand aascns = new AimAtSpeakerCommand(sh, e, k, false);

        Command aimAndShoot = scg.alongWith(aascns).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        Command aimAndSpin = aasc.deadlineWith(spinUp);

        addCommands(aimAndSpin, aimAndShoot);
    }

}
