package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Diagnostics;

public class RunDiagnosticsCommand extends CommandBase
{
    Diagnostics diagnosics;
    public RunDiagnosticsCommand(Diagnostics diag)
    {
        addRequirements(diag);
        diagnosics = diag;
    }
    @Override
    public void initialize() {
        System.out.println("diagnosic initialize");
    }

    @Override
    public void execute() {
        diagnosics.run();
        System.out.println("diagnosic execute");
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;

    }
    @Override
    public boolean runsWhenDisabled() {
        return true;

    }
}
