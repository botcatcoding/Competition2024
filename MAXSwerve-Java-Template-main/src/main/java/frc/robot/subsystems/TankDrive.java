package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {
//    CANSparkMax shoot;
    TalonFX frontLeft;
    TalonFX frontRight;
    TalonFX backLeft;
    TalonFX backRight;
    public TankDrive()
    {
        frontLeft = new TalonFX(0);
        frontRight = new TalonFX(1);
        backLeft = new TalonFX(2);
        backRight = new TalonFX(3);
        frontLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);
    }
    public void drive(double left, double right)
    {
        frontLeft.set(TalonFXControlMode.PercentOutput,left);
        backLeft.set(TalonFXControlMode.PercentOutput,left);
        frontRight.set(TalonFXControlMode.PercentOutput,-right);
        backRight.set(TalonFXControlMode.PercentOutput,-right);
        // shoot.set(speed);
    }
}
