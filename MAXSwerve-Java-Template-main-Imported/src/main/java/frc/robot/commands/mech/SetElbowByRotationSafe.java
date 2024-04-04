package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

//move to the position specified, but only if the shoulder is inside the "safe zone"
public class SetElbowByRotationSafe extends Command {

    Elbow elbow;
    Shoulder shoulder;
    double position;
    double shoulderSafeZone;
    boolean shouldEnd;
    double deadband;
    boolean inverted;
    boolean canInvert = false;

    public SetElbowByRotationSafe(double pos, boolean se, double dead, double shoulderSafeZon, boolean inv,
            Shoulder sh, Elbow e) {
        addRequirements(e);
        elbow = e;
        shoulder = sh;
        position = pos;
        shoulderSafeZone = shoulderSafeZon;
        position = pos;
        shouldEnd = se;
        deadband = dead;
        inverted = inv;
        if (inverted) {
            position = -position;
            // SmartDashboard.putNumber(getName(), pos)
        }
        // SmartDashboard.putBoolean("einverted", inverted);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double shoulderAbsoluteAngle = Math.abs(shoulder.getShoulderAngle() - Constants.MechConstants.shoulderMiddle);
        if (shoulderAbsoluteAngle < shoulderSafeZone) {// we can move because we are in the safe zone
            elbow.setPosition(position);
            SmartDashboard.putString("safety", "SAFE");
        } else {// we cant move rn
            elbow.dontMove();
            SmartDashboard.putString("safety", "UNSAFE");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // elbow.stop();
        // elbow.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return elbow.isPostioned(position, deadband) && shouldEnd;
    }
}
