package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Slurp;

public class SeekAndSlurpCommand extends Command {
    DriveSubsystem driveSubsystem;
    Slurp slurp;
    // Arm arm;

    // public SeekAndSlurpCommand(DriveSubsystem ds, Slurp sl, Arm a) {
    // addRequirements(sl);
    // driveSubsystem = ds;
    // slurp = sl;
    // // arm = a;
    // }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // limelightlib
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
