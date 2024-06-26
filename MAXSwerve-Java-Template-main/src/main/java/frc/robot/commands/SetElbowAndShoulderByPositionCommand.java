package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;


public class SetElbowAndShoulderByPositionCommand extends CommandBase
{

    double speed = 0;
    Arm arm;
    double elbowAngle;
    double shoulderAngle;

    public SetElbowAndShoulderByPositionCommand(double sa, double ea, Arm a)
    {
        addRequirements(a);
        arm = a;
        elbowAngle = ea;
        shoulderAngle = sa;
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        arm.setPosition( shoulderAngle, elbowAngle);
    }
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
    @Override
    public boolean isFinished() {
        return arm.isPostioned(shoulderAngle, elbowAngle);
    }

    
}