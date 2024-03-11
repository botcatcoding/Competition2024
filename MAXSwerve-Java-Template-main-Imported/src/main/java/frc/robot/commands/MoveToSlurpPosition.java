package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class MoveToSlurpPosition extends SequentialCommandGroup {

    public MoveToSlurpPosition(boolean inverted, Shoulder s, Elbow e, DriveSubsystem ds) {

        // indivual commands
        SetShoulderByRotation shoulderToSafe = new SetShoulderByRotation(
                Constants.MechConstants.shoulderMiddle + Constants.MechConstants.shoulderSafeZone
                        - Constants.MechConstants.shoulderSaveZoneEdge,
                false, inverted, ds,
                s);

        double slurpElbow = .09;
        double slurpShoulder = .52;
        SetElbowByRotationSafe elbowToSlurp = new SetElbowByRotationSafe(slurpElbow, true, .1,
                Constants.MechConstants.shoulderSafeZone, inverted, ds, s, e);
        SetElbowByRotationSafe elbowToSlurpNoEnd = new SetElbowByRotationSafe(slurpElbow, false, .1,
                Constants.MechConstants.shoulderSafeZone, inverted, ds, s, e);
        SetShoulderByRotation shoulderToSlurp = new SetShoulderByRotation(slurpShoulder, false, inverted, ds, s);

        // commands running at the same time
        Command shouldToSafeDeadline = Commands.deadline(elbowToSlurp, shoulderToSafe);
        Command shoulderToSlurpAndElbowToSlurp = Commands.parallel(shoulderToSlurp, elbowToSlurpNoEnd);

        // sequenced
        addCommands(shouldToSafeDeadline, shoulderToSlurpAndElbowToSlurp);
    }
}
