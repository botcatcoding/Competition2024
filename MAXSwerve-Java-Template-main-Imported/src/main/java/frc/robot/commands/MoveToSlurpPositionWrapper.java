package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class MoveToSlurpPositionWrapper extends Command {
    MoveToSlurpPosition mtsp;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Joystick m_mechDriver;

    public MoveToSlurpPositionWrapper(Shoulder s, Elbow e, DriveSubsystem ds,
            Joystick js) {
        shoulder = s;
        elbow = e;
        m_robotDrive = ds;
        m_mechDriver = js;

    }

    @Override
    public void initialize() {
        boolean shouldInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        SmartDashboard.putBoolean("shouldInvert", shouldInvert);
        mtsp = new MoveToSlurpPosition(shouldInvert, shoulder, elbow, m_robotDrive, m_mechDriver);
        mtsp.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        mtsp.cancel();
    }

    @Override
    public boolean isFinished() {
        return mtsp.isFinished();
    }
}
