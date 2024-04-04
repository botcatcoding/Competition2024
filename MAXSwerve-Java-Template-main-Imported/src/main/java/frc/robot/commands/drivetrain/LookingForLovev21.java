package frc.robot.commands.drivetrain;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LookingForLovev21 extends Command {

    PhotonCamera camera = new PhotonCamera("camera");
    DriveSubsystem driveSubsystem;

    PIDController turnController;
    double maximumX = 8;

    public LookingForLovev21(DriveSubsystem ds) {
        addRequirements(ds);
        driveSubsystem = ds;
        // Constraints fowardConstraints = new Constraints(1, 1);

        turnController = new PIDController(.05, 0.02, 0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("hasTargets", result.hasTargets());
        if (result.hasTargets()) {
            double ntx = result.targets.get(0).getYaw();
            // double ntx = LimelightHelpers.getTX(Constants.aOrangelightName);
            // double nty = LimelightHelpers.getTY(Constants.aOrangelightName);
            double forwardDrive = 0;
            if (Math.abs(ntx) < maximumX) {
                forwardDrive = -1.5;

            }
            double turn = turnController.calculate(ntx, 0);

            // double forwardDrive = 1 - Math.abs(ntx) / 10;

            // double rotation = Math.abs(ntx)/

            SmartDashboard.putNumber("Forward Drive", forwardDrive);
            SmartDashboard.putNumber("note X", ntx);
            SmartDashboard.putNumber("Turning", turn);
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(forwardDrive, 0, turn));
        } else {
            driveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
