package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    // double velocity = 0;
    double speed = 0;

    Shooter shooter;

    public ShootCommand(double s, Shooter sh)

    {
        addRequirements(sh);
        // velocity = v;
        speed = s;
        shooter = sh;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("shootCommand", speed);
        shooter.setShooter(speed, -speed / 2);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
