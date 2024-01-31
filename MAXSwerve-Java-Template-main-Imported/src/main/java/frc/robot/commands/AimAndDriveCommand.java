package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;

public class AimAndDriveCommand extends Command {
    Arm arm;
    DriveSubsystem driveSubsystem;
    Joystick joystick;
    PIDController yawPidController;

    public AimAndDriveCommand(DriveSubsystem ds, Arm a, Joystick j) {
        addRequirements(ds, a);
        arm = a;
        driveSubsystem = ds;
        joystick = j;
        yawPidController = new PIDController(
                AutoConstants.kPTheta, 0, 0);
        yawPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

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
        KinematicsResult kr = calculateArmKinematics(tx, ty, tz, pose.getX(), pose.getY(), 0, shoulderAngle);

        System.out.println(Rotation2d.fromRadians(shoulderAngle).getDegrees() + "\t"
                + Rotation2d.fromRadians(kr.elbowAngle).getDegrees() + "\t"
                + Rotation2d.fromRadians(kr.yaw).getDegrees());
        arm.setPosition(shoulderAngle, kr.elbowAngle);

        double dZ = yawPidController.calculate(driveSubsystem.getHeading(), kr.yaw);
        if (Math.abs(joystick.getZ()) > .5) {
            dZ = -MathUtil.applyDeadband(joystick.getZ(), OIConstants.kDriveDeadband);
        }
        double dX = -MathUtil.applyDeadband(joystick.getX(), OIConstants.kDriveDeadband);
        double dY = -MathUtil.applyDeadband(joystick.getY(), OIConstants.kDriveDeadband);
        driveSubsystem.drive(dX, dY, dZ, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

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

    static class KinematicsResult {
        double yaw;
        double elbowAngle;

        public KinematicsResult(double y, double e) {
            yaw = y;
            elbowAngle = e;
        }

    }
}
