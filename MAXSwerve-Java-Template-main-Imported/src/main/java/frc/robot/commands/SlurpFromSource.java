package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class SlurpFromSource extends ParallelCommandGroup {

    public SlurpFromSource(boolean inverted, Shoulder s, Elbow e, DriveSubsystem ds, Slurp slurp,
            DigitalInput slurpDetect) {

        double shoulderAngle = .19;
        double elbowAngle = 0;
        // indivual commands
        SetShoulderByRotation shoulderMovement = new SetShoulderByRotation(
                shoulderAngle,
                false, inverted, ds,
                s);

        SetElbowByRotationSafe elbowMovement = new SetElbowByRotationSafe(elbowAngle, false, .05,
                Constants.MechConstants.shoulderSafeZone, inverted, ds, s, e);
        SlurpCommand slurpCommand = new SlurpCommand(-6000, true, true, slurp, slurpDetect);
        // commands running at the same time
        addCommands(slurpCommand, shoulderMovement, elbowMovement);

    }
}
