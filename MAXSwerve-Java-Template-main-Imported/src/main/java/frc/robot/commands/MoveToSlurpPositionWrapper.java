package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class MoveToSlurpPositionWrapper extends Command {
    MoveToSlurpPosition mtsp;
    SlurpCommand slurpCommand;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Slurp slurp;
    Joystick m_mechDriver;
    DigitalInput slurpdetect;
    boolean shouldInvert;

    public MoveToSlurpPositionWrapper(Shoulder s, Elbow e, DriveSubsystem ds,
            Joystick js, DigitalInput sd, Slurp sl) {
        shoulder = s;
        elbow = e;
        m_robotDrive = ds;
        m_mechDriver = js;
        slurpdetect = sd;
        slurp = sl;

    }

    @Override
    public void initialize() {
        shouldInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        // SmartDashboard.putBoolean("shouldInvert", shouldInvert);
        mtsp = new MoveToSlurpPosition(shouldInvert, shoulder, elbow, m_robotDrive);
        mtsp.schedule();
        slurpCommand = new SlurpCommand(-6000, true, true, slurp, slurpdetect);
        slurpCommand.schedule();
    }

    @Override
    public void execute() {
        boolean nowInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        if (shouldInvert != nowInvert) {
            shouldInvert = nowInvert;
            mtsp.cancel();
            mtsp = new MoveToSlurpPosition(shouldInvert, shoulder, elbow, m_robotDrive);
            mtsp.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mtsp.cancel();
        slurpCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
