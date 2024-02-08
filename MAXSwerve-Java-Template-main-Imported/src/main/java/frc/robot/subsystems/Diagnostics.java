package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Diagnostics extends SubsystemBase {
    private DriveSubsystem m_robotDrive;

    public Diagnostics(DriveSubsystem ds) {
        m_robotDrive = ds;
    }

    @Override
    public void periodic() {
        if (true) {
            return;
        }
        SmartDashboard.putNumber("test",
                m_robotDrive.m_frontLeft.m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        String drivetrainFaults = m_robotDrive.diagnostic();
        if (drivetrainFaults == null) {
            SmartDashboard.putString("drivetrainFaults", "Drivetrain Healthy");
        } else {
            SmartDashboard.putString("drivetrainFaults", drivetrainFaults);
        }
    }

    // public static String diagnoseFalcon(TalonFX theMotor) {
    // theMotor.getFaultField().refresh();
    // theMotor.getFaultField().getStatus();

    // }
}
