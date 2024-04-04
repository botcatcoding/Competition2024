package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.Kinamatics;

public class ShootCommandWithLength extends Command {
    double velocity = 0;
    Shooter shooter;
    boolean shouldstop;
    Kinamatics kinematics;

    public ShootCommandWithLength(Shooter sh, Kinamatics kin, boolean ss) {

        addRequirements(sh);
        kinematics = kin;
        shooter = sh;
        shouldstop = ss;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("shootCommand", speed);
        if (kinematics.length < 2) {
            shooter.setShooterVelocity(Constants.MechConstants.shootSpeedClose,
                    Constants.MechConstants.shootSpeedClose);
        } else {
            shooter.setShooterVelocity(Constants.MechConstants.shootSpeed, Constants.MechConstants.shootSpeed);
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
