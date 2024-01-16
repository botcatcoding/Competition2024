package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
//    CANSparkMax shoot;
    TalonFX shootl;
    TalonFX shootr;
    public Shooter()
    {
    //    shoot = new CANSparkMax(21, MotorType.kBrushless);
       shootl = new TalonFX(10);
       shootr = new TalonFX(11);
       shootl.setNeutralMode(NeutralMode.Coast);
       shootr.setNeutralMode(NeutralMode.Coast);
    }
    public void setShooter(double speedl,double speedr)
    {
        shootl.set(TalonFXControlMode.PercentOutput,speedl);
        shootr.set(TalonFXControlMode.PercentOutput,-speedr);
        // shoot.set(speed);
    }
}
