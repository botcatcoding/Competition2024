package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kinamatics extends SubsystemBase {
    DriveSubsystem driveSubsystem;
    double tx;
    double ty;
    double tz;

    public double elbowRotation;
    public double shoulderRotation;
    static public double yaw;

    public double length;

    public Kinamatics(DriveSubsystem ds) {
        driveSubsystem = ds;
    }

    @Override
    public void periodic() {

        tx = Constants.Arena.blueTargetX;
        ty = Constants.Arena.blueTargetY;
        tz = Constants.Arena.blueTargetZ;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
                tx = Constants.Arena.redTargetX;
                ty = Constants.Arena.redTargetY;
                tz = Constants.Arena.redTargetZ;
            }
        }
        Pose2d pose = driveSubsystem.getPose();
        double shoulderAngle = Rotation2d.fromDegrees(110).getRadians();

        double differenceY = ty - pose.getY();
        double differenceX = tx - pose.getX();

        length = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));

        double noteMetersPerSecond = 12;
        // double scaler = .75;
        // double whenToStartAdding = 3;
        // if (l > whenToStartAdding) {
        // noteMetersPerSecond = noteMetersPerSecond + (l * scaler - whenToStartAdding);
        // }

        double flightTime = length / noteMetersPerSecond;

        ChassisSpeeds fieldRelative = driveSubsystem.getFieldRelativeSpeeds();

        double AimAheadtx = tx - fieldRelative.vxMetersPerSecond * flightTime;
        double AimAheadty = ty - fieldRelative.vyMetersPerSecond * flightTime;
        double BounceShottz = tz;
        if (length < 2) {
            BounceShottz = BounceShottz + .15;
        }
        // double AimUpTz = tz + l / 6;

        KinematicsResult krBeforeDrop = calculateArmKinematics(AimAheadtx, AimAheadty, BounceShottz, pose.getX(),
                pose.getY(), 0, shoulderAngle);
        double theta = krBeforeDrop.elbowAngle - Math.PI / 2;
        double time = length / (noteMetersPerSecond * Math.cos(theta));
        SmartDashboard.putNumber("time", time);
        SmartDashboard.putNumber("absDrop", 0.5 * (9.81) * Math.pow(time, 2));
        double drop = 0.5 * (9.81) * Math.pow(time, 2);
        SmartDashboard.putNumber("drop", drop);

        KinematicsResult kr = calculateArmKinematics(AimAheadtx, AimAheadty, BounceShottz + drop, pose.getX(),
                pose.getY(), 0, shoulderAngle);

        elbowRotation = kr.elbowAngle / (2 * Math.PI);
        shoulderRotation = shoulderAngle / (2 * Math.PI);

        yaw = kr.yaw;
        // SmartDashboard.putNumber("KinShoulder", shoulderAngle);
        SmartDashboard.putNumber("KinElbow", elbowRotation);
        SmartDashboard.putNumber("KinYaw", yaw);

    }

    public static KinematicsResult calculateArmKinematics(double tx, double ty, double tz, double bx, double by,
            double bz, double sa) {
        double differenceY = ty - by;
        double differenceX = tx - bx;
        SmartDashboard.putNumber("diffY", differenceY);
        SmartDashboard.putNumber("diffX", differenceX);
        double yaw = Math.atan2(differenceY, differenceX);
        double shoulderZ = bz + Constants.MechConstants.kShoulderZOffset;
        double d = Math.cos(sa) * Constants.MechConstants.kShoulderLength;
        double l = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));
        double e = l - d;
        double f = Math.sin(sa) * Constants.MechConstants.kShoulderLength + shoulderZ;
        double distanceToAimUp = 2;
        SmartDashboard.putNumber("length", l);
        if (l > distanceToAimUp && (l < 3)) {
            tz = tz + (l - distanceToAimUp) / 12;
        }
        double differenceZ = tz - bz;
        double g = differenceZ - f;
        // System.out.println(e + "\t" + g);
        SmartDashboard.putNumber("calculated Elbow", Rotation2d.fromRadians(Math.atan2(g, e)).getDegrees());
        double toElbow = Math.atan2(g, e) + Math.PI / 2;
        double offset = Rotation2d.fromDegrees(90).getRadians() - sa;
        double elbowAngle = toElbow + offset;

        return new KinematicsResult(yaw, elbowAngle);
    }

    public static class KinematicsResult {
        public double yaw;
        public double elbowAngle;

        public KinematicsResult(double y, double e) {
            yaw = y;
            elbowAngle = e;
        }

    }
}
