package frc.robot.commands;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    DriveSubsystem driveSubsystem;
    Joystick joystick;

    public DriveCommand(DriveSubsystem ds, Joystick j) {
        addRequirements(ds);
        driveSubsystem = ds;
        joystick = j;

    }

    @Override
    public void initialize() {
        driveSubsystem.pointAtYaw = false;
    }

    @Override
    public void execute() {

        double yawCommand = -MathUtil.applyDeadband(
                (joystick.getZ() * joystick.getZ() * Math.signum(joystick.getZ())) / 2, OIConstants.kDriveDeadband / 2);
        driveSubsystem.drive(
                -MathUtil.applyDeadband(joystick.getY() * joystick.getY() * Math.signum(joystick.getY()),
                        OIConstants.kDriveDeadband / 2),
                -MathUtil.applyDeadband(joystick.getX() * joystick.getX() * Math.signum(joystick.getX()),
                        OIConstants.kDriveDeadband / 2),
                yawCommand, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
