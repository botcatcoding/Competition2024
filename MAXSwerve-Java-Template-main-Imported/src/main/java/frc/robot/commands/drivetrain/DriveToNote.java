package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToNote extends Command {
    DriveSubsystem driveSubsystem;
    PIDController forwardController;
    PIDController turnController;

    double targetY = -12;
    double maximumX = 5;

    public DriveToNote(DriveSubsystem ds) {
        addRequirements(ds);
        driveSubsystem = ds;
        // Constraints fowardConstraints = new Constraints(1, 1);
        forwardController = new PIDController(.05, 0.01, 0);

        turnController = new PIDController(.1, 0.05, 0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double ntx = LimelightHelpers.getTX(Constants.aOrangelightName);
        double nty = LimelightHelpers.getTY(Constants.aOrangelightName);
        double forwardDrive = 0;
        if (Math.abs(ntx) < maximumX) {
            forwardDrive = forwardController.calculate(nty, targetY);

        }

        double turn = turnController.calculate(ntx, 0);

        // double forwardDrive = 1 - Math.abs(ntx) / 10;

        // double rotation = Math.abs(ntx)/

        SmartDashboard.putNumber("Forward Drive", forwardDrive);
        SmartDashboard.putNumber("note X", ntx);
        SmartDashboard.putNumber("note Y", nty);
        SmartDashboard.putNumber("Turning", turn);
        driveSubsystem.driveRobotRelative(new ChassisSpeeds(forwardDrive, 0, turn));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX(Constants.aOrangelightName)) < 0.2
                && Math.abs(LimelightHelpers.getTY(Constants.aOrangelightName) - targetY) < 2;
    }
}
