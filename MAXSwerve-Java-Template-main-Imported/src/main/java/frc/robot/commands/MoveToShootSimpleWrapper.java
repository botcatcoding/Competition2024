package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class MoveToShootSimpleWrapper extends Command {
    MoveToShootSimple mtss;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Slurp slurp;
    Joystick m_mechDriver;
    boolean shouldInvert;

    public MoveToShootSimpleWrapper(Shoulder s, Elbow e, DriveSubsystem ds) {
        shoulder = s;
        elbow = e;
        m_robotDrive = ds;

    }

    @Override
    public void initialize() {
        shouldInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        mtss = new MoveToShootSimple(shouldInvert, shoulder, elbow, m_robotDrive);
        mtss.schedule();
    }

    @Override
    public void execute() {
        boolean nowInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        if (shouldInvert != nowInvert) {
            shouldInvert = nowInvert;
            mtss.cancel();
            mtss = new MoveToShootSimple(shouldInvert, shoulder, elbow, m_robotDrive);
            mtss.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mtss.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
