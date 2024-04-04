package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class YawToSpeaker extends Command {
    DriveSubsystem driveSubsystem;
    boolean wc;

    public YawToSpeaker(DriveSubsystem ds, boolean whileChoreo) {
        wc = whileChoreo;
        if (!whileChoreo) {
            addRequirements(ds);
        }
        driveSubsystem = ds;

    }

    @Override
    public void initialize() {
        driveSubsystem.resetProfiled();
    }

    @Override
    public void execute() {
        driveSubsystem.setPointAtYaw(true);
        if (!wc) {
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {

        driveSubsystem.setPointAtYaw(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
