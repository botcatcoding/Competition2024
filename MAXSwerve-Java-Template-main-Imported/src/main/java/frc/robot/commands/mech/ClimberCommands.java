package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class ClimberCommands {
    public static class ClimbUpGroup extends SequentialCommandGroup {
        public ClimbUpGroup(Shoulder sho, Elbow e, boolean inverted) {

            SetElbowByRotationSafe moveElbow = new SetElbowByRotationSafe(.35, false,
                    Constants.MechConstants.elbowDeadband,
                    Constants.MechConstants.shoulderSafeZone, inverted, sho, e);
            ChangeElbowConfiguration cec = new ChangeElbowConfiguration(e, false, false);
            addCommands(cec, moveElbow);
        }
    }

    public static class AintNoMountainHighEnough extends SequentialCommandGroup {
        public AintNoMountainHighEnough(Shoulder sho, Elbow e, boolean inverted) {

            SetElbowByRotationSafe moveElbow = new SetElbowByRotationSafe(-.05, true,
                    Constants.MechConstants.elbowDeadband,
                    Constants.MechConstants.shoulderSafeZone, inverted, sho, e);
            SetElbowByRotationSafe moveElbow2 = new SetElbowByRotationSafe(-.05, false,
                    Constants.MechConstants.elbowDeadband,
                    Constants.MechConstants.shoulderSafeZone, inverted, sho, e);
            SetShoulderByRotation moveShoulder = new SetShoulderByRotation(.05, inverted, inverted, sho,
                    Constants.MechConstants.shoulderDeadband);
            ChangeElbowConfiguration cec = new ChangeElbowConfiguration(e, false, false);
            ChangeShoulderConfiguration csc = new ChangeShoulderConfiguration(sho, false, false);
            addCommands(cec, csc, moveElbow, moveShoulder.alongWith(moveElbow2));
        }
    }

    public static class ClimbOver extends SequentialCommandGroup {
        public ClimbOver(Shoulder sho, Elbow e) {
            WaitCommand wait = new WaitCommand(1);
            ChangeElbowConfiguration cec = new ChangeElbowConfiguration(e, true, true);
            ChangeShoulderConfiguration csc = new ChangeShoulderConfiguration(sho, false, true);
            addCommands(wait, cec, csc);
        }
    }
}
