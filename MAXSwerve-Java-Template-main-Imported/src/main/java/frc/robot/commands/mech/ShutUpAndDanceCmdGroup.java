package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.ShutUpAndDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class ShutUpAndDanceCmdGroup extends DebugedSequentialCommandGroup {
    public ShutUpAndDanceCmdGroup(Lighting lighting, DriveSubsystem ds, Elbow e, Shoulder sh, DigitalInput sd, Slurp sl,
            Shooter sho) {
        MoveToSlurpPositionWrapper.MoveToSlurpPosition mtsp1 = new MoveToSlurpPositionWrapper.MoveToSlurpPosition(false,
                true, sh, e, lighting);
        TheSmallGroup sg = new TheSmallGroup(lighting, sl, sho, e, sh, sd, false);
        ShutUpAndDrive shut = new ShutUpAndDrive(-.75, 0, 0, ds);
        WaitCommand watchDog = new WaitCommand(Math.PI);

        addCommands(mtsp1, watchDog.deadlineWith(shut, sg));
    }
}
