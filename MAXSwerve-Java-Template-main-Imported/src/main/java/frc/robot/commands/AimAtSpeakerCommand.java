package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonFormat.Feature;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm.KinematicsResult;

public class AimAtSpeakerCommand extends Command {
    Arm arm;
    DriveSubsystem driveSubsystem;

    public AimAtSpeakerCommand(DriveSubsystem ds, Arm a) {
        addRequirements(a);
        arm = a;
        driveSubsystem = ds;
    }

    @Override
    public void initialize() {
        driveSubsystem.pointAtYaw = true;

    }

    @Override
    public void execute() {

        boolean isRedTeam = false;
        double tx = Constants.Arena.blueTargetX;
        double ty = Constants.Arena.blueTargetY;
        double tz = Constants.Arena.blueTargetZ;
        if (isRedTeam) {
            tx = Constants.Arena.redTargetX;
            ty = Constants.Arena.redTargetY;
            tz = Constants.Arena.redTargetZ;
        }
        Pose2d pose = driveSubsystem.getPose();
        double shoulderAngle = Rotation2d.fromDegrees(90).getRadians();

        double noteMetersPerSecond = 2;

        double differenceY = ty - pose.getY();
        double differenceX = tx - pose.getX();

        double l = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));

        double flightTime = l / noteMetersPerSecond;

        ChassisSpeeds fieldRelative = driveSubsystem.getFieldRelativeSpeeds();

        tx = tx - fieldRelative.vxMetersPerSecond * flightTime;
        ty = ty - fieldRelative.vyMetersPerSecond * flightTime;

        KinematicsResult kr = Arm.calculateArmKinematics(tx, ty, tz, pose.getX(), pose.getY(), 0, shoulderAngle);

        // System.out.println(Rotation2d.fromRadians(shoulderAngle).getDegrees() + "\t"
        // + Rotation2d.fromRadians(kr.elbowAngle).getDegrees() + "\t"
        // + Rotation2d.fromRadians(kr.yaw).getDegrees());
        arm.setPosition(shoulderAngle, kr.elbowAngle);
        driveSubsystem.theYaw = kr.yaw;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        driveSubsystem.pointAtYaw = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
