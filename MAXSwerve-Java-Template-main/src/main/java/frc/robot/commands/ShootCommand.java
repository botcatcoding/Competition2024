package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase{
    double speedl = 0;
    double speedr = 0;

    Shooter shooter;
    public ShootCommand(double sl, double sr , Shooter sh)

        {
            addRequirements(sh);
            speedl = sl;
            speedr = sr;
            shooter = sh;
        }
        @Override
        public void initialize() {
            
        }

        @Override
        public void execute() {
            shooter.setShooter(speedl,speedr);
        }
        @Override
        public void end(boolean interrupted) {

        }
        @Override
        public boolean isFinished() {
            return false;
        }
}
