package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shoulder;

public class AimAtSpeakerCommand extends Command {
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem driveSubsystem;

    Kinamatics kinamatics;
    boolean shouldStop;
    double dd;

    public AimAtSpeakerCommand(DriveSubsystem ds, Shoulder sh, Elbow e, Kinamatics k, boolean ss,
            double driveDeadband) {
        addRequirements(sh, e);
        elbow = e;
        shoulder = sh;
        driveSubsystem = ds;
        dd = driveDeadband;
        kinamatics = k;
        shouldStop = ss;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shoulder.setPosition(kinamatics.shoulderRotation);
        double shoulderAbsoluteAngle = Math.abs(shoulder.getShoulderAngle() - Constants.MechConstants.shoulderMiddle);
        if (shoulderAbsoluteAngle < Constants.MechConstants.shoulderSafeZone) {// we can move because we are in the safe
                                                                               // zone
            elbow.setPosition(kinamatics.elbowRotation);

        } else {// we cant move rn
            elbow.dontMove();

        }
        // SmartDashboard.putNumber("targetYaw",
        // Rotation2d.fromRadians(kr.yaw).getDegrees());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        double elbowDead = Constants.MechConstants.elbowDeadband;
        double shoulderDeadband = Constants.MechConstants.shoulderDeadband;
        if (kinamatics.length < 2) {
            elbowDead = elbowDead * 2;
            shoulderDeadband = shoulderDeadband * 2;
        }
        return shouldStop && shoulder.isPostioned(kinamatics.shoulderRotation, shoulderDeadband)
                && elbow.isPostioned(kinamatics.elbowRotation, elbowDead) && driveSubsystem.isPostioned(dd);
    }

}
