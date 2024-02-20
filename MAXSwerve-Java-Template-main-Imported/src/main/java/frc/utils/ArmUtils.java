package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmUtils {

    public static KinematicsResult calculateArmKinematics(double tx, double ty, double tz, double bx, double by,
            double bz, double sa) {
        double differenceY = ty - by;
        double differenceX = tx - bx;
        double differenceZ = tz - bz;
        double yaw = Math.asin(differenceY / differenceX);
        double shoulderZ = bz + Constants.MechConstants.kShoulderZOffset;
        double d = Math.cos(sa) / Constants.MechConstants.kShoulderLength;
        double l = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));
        double e = l - d;
        double f = Math.sin(sa) * Constants.MechConstants.kShoulderLength + shoulderZ;
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
