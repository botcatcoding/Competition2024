package frc.robot.commands.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowGenedTraj extends Command {
    DriveSubsystem driveSubsystem;
    PathPlannerPath path;
    Command followCommand;

    public FollowGenedTraj(DriveSubsystem ds) {
        driveSubsystem = ds;
    }

    @Override
    public void initialize() {
        Pose2d startPose = driveSubsystem.getPose();
        Pose2d endPose = new Pose2d(startPose.getX() + 1, startPose.getY(), startPose.getRotation());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                new GoalEndState(0.0, new Rotation2d(Math.PI)));

        // Prevent this path from being flipped on the red alliance, since the given
        // positions are already correct
        path.preventFlipping = true;
        followCommand = AutoBuilder.followPath(path);
        followCommand.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return followCommand.isFinished();
    }
}
