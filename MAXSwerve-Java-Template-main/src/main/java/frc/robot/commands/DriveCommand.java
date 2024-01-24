package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{
    DriveSubsystem driveSubsystem;
    Joystick joystick;
    public DriveCommand(DriveSubsystem ds,Joystick j)
    {
        driveSubsystem = ds;
        joystick = j;



    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
                -MathUtil.applyDeadband(joystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getZ(), OIConstants.kDriveDeadband),
                true, true);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
