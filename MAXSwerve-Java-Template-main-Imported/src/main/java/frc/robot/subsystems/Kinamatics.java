package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public Kinamatics(DriveSubsystem ds, boolean isRedAlliance) {
        driveSubsystem = ds;
        tx = Constants.Arena.blueTargetX;
        ty = Constants.Arena.blueTargetY;
        tz = Constants.Arena.blueTargetZ;
        if (isRedAlliance) {
            tx = Constants.Arena.redTargetX;
            ty = Constants.Arena.redTargetY;
            tz = Constants.Arena.redTargetZ;
        }
    }

    @Override
    public void periodic() {

        Pose2d pose = driveSubsystem.getPose();
        double shoulderAngle = Rotation2d.fromDegrees(90).getRadians();

        double noteMetersPerSecond = 4;

        double differenceY = ty - pose.getY();
        double differenceX = tx - pose.getX();

        double l = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));

        double flightTime = l / noteMetersPerSecond;

        ChassisSpeeds fieldRelative = driveSubsystem.getFieldRelativeSpeeds();

        double AimAheadtx = tx + fieldRelative.vxMetersPerSecond * flightTime;
        double AimAheadty = ty + fieldRelative.vyMetersPerSecond * flightTime;

        KinematicsResult kr = calculateArmKinematics(AimAheadtx, AimAheadty, tz, pose.getX(),
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
        double d = Math.cos(sa) / Constants.MechConstants.kShoulderLength;
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
        // System.out.println(e+"\t"+g);
        double elbowAngle = Rotation2d.fromDegrees(180).getRadians() - sa + Math.atan2(g, e);

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
