package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class SetShoulderByRotation extends Command {
    Shoulder shoulder;
    DriveSubsystem driveSubsystem;
    Joystick joystick;
    double position;
    boolean shouldEnd;
    boolean inverted;
    boolean canInvert = false;

    double invertPosition(double in) {
        return Constants.MechConstants.shoulderMiddle - (in - Constants.MechConstants.shoulderMiddle);
    }

    public SetShoulderByRotation(double pos, boolean se, boolean inv, Joystick js, DriveSubsystem ds,
            Shoulder sh) {
        addRequirements(sh);
        shoulder = sh;
        driveSubsystem = ds;
        joystick = js;
        position = pos;
        shouldEnd = se;
        inverted = inv;
        canInvert = true;
        if (inverted) {
            position = invertPosition(position);
            // SmartDashboard.putNumber(getName(), pos)
        }
        SmartDashboard.putBoolean("sinverted", inverted);
    }

    public SetShoulderByRotation(double pos, boolean se, boolean inv,
            Shoulder sh) {
        addRequirements(sh);
        shoulder = sh;
        position = pos;
        shouldEnd = se;
        inverted = inv;
        canInvert = false;
        if (inverted) {
            position = invertPosition(position);
            // SmartDashboard.putNumber(getName(), pos)
        }
        SmartDashboard.putBoolean("sinverted", inverted);
    }

    @Override
    public void initialize() {
        // inverted = Slurp.slurpFieldOrient(joystick.getPOV(),
        // driveSubsystem.getHeading());
    }

    @Override
    public void execute() {
        if (canInvert) {
            boolean nowInverted = Slurp.slurpFieldOrient(joystick.getPOV(),
                    driveSubsystem.getHeading());
            if (nowInverted != inverted) {
                cancel();
            }
        }
        shoulder.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.stop();
    }

    @Override
    public boolean isFinished() {
        return shoulder.isPostioned(position) && shouldEnd;

    }
}
