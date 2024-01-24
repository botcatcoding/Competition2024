
    package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Feed extends SubsystemBase {
    CANSparkMax IntakeFL;
    CANSparkMax IntakeFR;
    CANSparkMax IntakeBL;
    CANSparkMax IntakeBR;
//    TalonFX feedl;
//    TalonFX feedr;
    public Feed()
    {
        IntakeFL = new CANSparkMax(12, MotorType.kBrushless);
        IntakeFR = new CANSparkMax(11, MotorType.kBrushless);
        IntakeBL = new CANSparkMax(9, MotorType.kBrushless);
        IntakeBR = new CANSparkMax(10, MotorType.kBrushless);

        
    //   feedl = new TalonFX(12); 
    //   feedr = new TalonFX(13);
    //   feedl.setNeutralMode(NeutralMode.Coast);
    //   feedr.setNeutralMode(NeutralMode.Coast);
    }
    public void setFeedVelo(int ticks)
    {

    }
    public void setFeedPercent(double speed)
    {

        IntakeFL.set(speed);
        IntakeBL.set(speed);
        

    //    feedl.set(TalonFXControlMode.PercentOutput,speed);
    //    feedr.set(TalonFXControlMode.PercentOutput,-speed);
    //    //feed.set(speed);
    }
}


