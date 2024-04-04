package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.drivetrain.LookingForLovev21;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;;

public class ShutUpAndDanceCmdGroup extends DebugedSequentialCommandGroup {
    public ShutUpAndDanceCmdGroup(Lighting lighting, DriveSubsystem ds, Elbow e, Shoulder sh, DigitalInput sd, Slurp sl,
            Shooter sho) {
        MoveToSlurpPositionWrapper.MoveToSlurpPosition mtsp1 = new MoveToSlurpPositionWrapper.MoveToSlurpPosition(false,
                true, sh, e, lighting);
        TheSmallGroup sg = new TheSmallGroup(lighting, sl, sho, e, sh, sd, false);
        LookingForLovev21 lfl = new LookingForLovev21(ds);

        addCommands(mtsp1, lfl.deadlineWith(sg));
    }
}
