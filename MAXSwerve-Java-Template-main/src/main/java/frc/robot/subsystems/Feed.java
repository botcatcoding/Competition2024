
    package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Feed extends SubsystemBase {
    // CANSparkMax feed;
   TalonFX feedl;
   TalonFX feedr;
    public Feed()
    {
        // feed = new CANSparkMax(20, MotorType.kBrushless);
      feedl = new TalonFX(12); 
      feedr = new TalonFX(13);
      feedl.setNeutralMode(NeutralMode.Coast);
      feedr.setNeutralMode(NeutralMode.Coast);
    }
    public void setFeed(double speed)
    {
       feedl.set(TalonFXControlMode.PercentOutput,speed);
       feedr.set(TalonFXControlMode.PercentOutput,-speed);
    //    feed.set(speed);
    }
}


