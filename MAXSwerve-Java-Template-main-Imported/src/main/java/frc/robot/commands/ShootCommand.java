package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    double velocity = 0;
    double speed = 0;
    Shooter shooter;
    boolean velocityMode;
    boolean shouldstop;

    public ShootCommand(Shooter sh, double s, boolean vm, boolean ss) {

        addRequirements(sh);
        velocity = s;
        shooter = sh;
        velocityMode = vm;
        shouldstop = ss;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("shootCommand", speed);
        if (velocityMode) {
            shooter.setShooterVelocity(velocity, velocity);
        } else {
            shooter.setShooterPercent(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed() && shouldstop;
    }
}
