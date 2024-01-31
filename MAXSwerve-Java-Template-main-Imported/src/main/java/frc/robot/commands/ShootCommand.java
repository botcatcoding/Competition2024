package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command{
    double velocity = 0;

    Shooter shooter;
    public ShootCommand(double v, Shooter sh)

        {
            addRequirements(sh);
            velocity = v;
            shooter = sh;
        }
        @Override
        public void initialize() {
            
        }

        @Override
        public void execute() {
            shooter.setShooter(velocity);
        }
        @Override
        public void end(boolean interrupted) {

        }
        @Override
        public boolean isFinished() {
            return false;
        }
}
