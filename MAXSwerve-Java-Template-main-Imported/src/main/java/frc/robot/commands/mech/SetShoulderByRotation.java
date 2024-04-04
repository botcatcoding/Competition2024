package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class SetShoulderByRotation extends Command {
    Shoulder shoulder;
    double position;
    boolean shouldEnd;
    boolean inverted;
    boolean canInvert = false;
    double deadband;

    double invertPosition(double in) {
        return Constants.MechConstants.shoulderMiddle - (in - Constants.MechConstants.shoulderMiddle);
    }

    public SetShoulderByRotation(double pos, boolean se, boolean inv,
            Shoulder sh, double db) {
        addRequirements(sh);
        shoulder = sh;
        position = pos;
        shouldEnd = se;
        inverted = inv;
        canInvert = false;
        deadband = db;
        if (inverted) {
            position = invertPosition(position);
            // SmartDashboard.putNumber(getName(), pos)
        }
        // SmartDashboard.putBoolean("sinverted", inverted);
    }

    @Override
    public void initialize() {
        // inverted = Slurp.slurpFieldOrient(joystick.getPOV(),
        // driveSubsystem.getHeading());
    }

    @Override
    public void execute() {
        shoulder.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        // shoulder.stop();
    }

    @Override
    public boolean isFinished() {
        return shoulder.isPostioned(position, deadband) && shouldEnd;

    }
}
