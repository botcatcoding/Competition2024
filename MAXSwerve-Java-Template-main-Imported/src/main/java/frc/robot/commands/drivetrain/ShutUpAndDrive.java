package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ShutUpAndDrive extends Command {
    DriveSubsystem driveSubsystem;
    ChassisSpeeds chassisSpeeds;

    public ShutUpAndDrive(double x, double y, double rot, DriveSubsystem ds) {

        addRequirements(ds);
        driveSubsystem = ds;
        chassisSpeeds = new ChassisSpeeds(x, y, rot);
    }

    @Override
    public void initialize() {
        driveSubsystem.theYaw = driveSubsystem.getPose().getRotation().getRadians();
        driveSubsystem.profiledPIDController.reset(driveSubsystem.theYaw);
        driveSubsystem.pointAtYaw = true;

    }

    @Override
    public void execute() {
        driveSubsystem.driveRobotRelative(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.pointAtYaw = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
