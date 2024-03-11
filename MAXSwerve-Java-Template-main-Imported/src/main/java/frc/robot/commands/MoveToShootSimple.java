package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class MoveToShootSimple extends ParallelCommandGroup {

    public MoveToShootSimple(boolean inverted, Shoulder s, Elbow e, DriveSubsystem ds) {

        double shoulderAngle = .3;
        double elbowAngle = .35;
        // indivual commands
        SetShoulderByRotation shoulderMovement = new SetShoulderByRotation(
                shoulderAngle,
                false, inverted, ds,
                s);

        SetElbowByRotationSafe elbowMovement = new SetElbowByRotationSafe(elbowAngle, true, .05,
                Constants.MechConstants.shoulderSafeZone, inverted, ds, s, e);

        // commands running at the same time
        addCommands(shoulderMovement, elbowMovement);

    }
}
