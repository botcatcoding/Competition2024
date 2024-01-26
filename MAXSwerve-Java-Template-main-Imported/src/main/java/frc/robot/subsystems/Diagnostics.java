package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Diagnostics extends SubsystemBase {
    private DriveSubsystem m_robotDrive;
    public Diagnostics(DriveSubsystem ds)
    {
        m_robotDrive = ds;
    }
    public void run() {
        m_robotDrive.diagnostic();
    }
}
