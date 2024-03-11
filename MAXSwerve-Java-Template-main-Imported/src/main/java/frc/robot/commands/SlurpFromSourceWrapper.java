package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class SlurpFromSourceWrapper extends Command {
    SlurpFromSource sfs;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Slurp slurp;
    boolean shouldInvert;
    DigitalInput slurpDetect;

    public SlurpFromSourceWrapper(Shoulder sh, Elbow e, DriveSubsystem ds, Slurp sl, DigitalInput sd) {

        shoulder = sh;
        elbow = e;
        slurp = sl;
        m_robotDrive = ds;
        slurpDetect = sd;

    }

    @Override
    public void initialize() {
        shouldInvert = Slurp.slurpFieldOrient(270, m_robotDrive.getHeading());
        sfs = new SlurpFromSource(shouldInvert, shoulder, elbow, m_robotDrive, slurp, slurpDetect);
        sfs.schedule();
        m_robotDrive.pointAtYaw = true;
        if (shouldInvert) {
            m_robotDrive.theYaw = Rotation2d.fromDegrees(120).getRadians();
        } else {
            m_robotDrive.theYaw = Rotation2d.fromDegrees(-60).getRadians();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        sfs.cancel();
        m_robotDrive.pointAtYaw = false;
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
