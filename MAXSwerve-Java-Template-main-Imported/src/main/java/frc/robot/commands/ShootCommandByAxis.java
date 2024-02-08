package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommandByAxis extends Command {
    // double velocity = 0;

    Shooter shooter;
    Joystick joystick;

    public ShootCommandByAxis(Joystick js, Shooter sh)

    {
        addRequirements(sh);
        // velocity = v;
        joystick = js;
        shooter = sh;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = -joystick.getRawAxis(3);
        speed = (speed + 1) / 2;
        shooter.setShooter(speed, -speed / 2);
        SmartDashboard.putNumber("shootSpeed", speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
