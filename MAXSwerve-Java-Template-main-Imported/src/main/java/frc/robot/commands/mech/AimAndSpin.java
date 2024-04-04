package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class AimAndSpin extends ParallelCommandGroup {
    public AimAndSpin(DriveSubsystem ds, Shoulder sh, Elbow e, Kinamatics k, Shooter sho) {
        ShootCommandWithLength spinUp = new ShootCommandWithLength(sho, k,
                false);
        AimAtSpeakerCommand aasc = new AimAtSpeakerCommand(ds, sh, e, k, false, 100);
        addCommands(aasc, spinUp);
    }
}
