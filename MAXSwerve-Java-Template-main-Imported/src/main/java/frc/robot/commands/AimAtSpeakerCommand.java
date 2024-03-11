package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AudioSubsytem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shoulder;

public class AimAtSpeakerCommand extends Command {
    Shoulder shoulder;
    Elbow elbow;

    Kinamatics kinamatics;
    boolean shouldStop;

    public AimAtSpeakerCommand(Shoulder sh, Elbow e, Kinamatics k, boolean ss) {
        addRequirements(sh, e);
        elbow = e;
        shoulder = sh;

        kinamatics = k;
        shouldStop = ss;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shoulder.setPosition(kinamatics.shoulderRotation);
        elbow.setPosition(kinamatics.elbowRotation);
        // SmartDashboard.putNumber("targetYaw",
        // Rotation2d.fromRadians(kr.yaw).getDegrees());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return shouldStop && shoulder.isPostioned(kinamatics.shoulderRotation, Constants.MechConstants.shoulderDeadband)
                && elbow.isPostioned(kinamatics.elbowRotation, Constants.MechConstants.elbowDeadband);
    }

}
