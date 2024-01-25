package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Feed extends SubsystemBase {
CANSparkMax intakeTL;
CANSparkMax intakeTR;
CANSparkMax intakeBL;
CANSparkMax intakeBR;

    private SparkPIDController intakeTLPID;
//    TalonFX feedl;
//    TalonFX feedr;
    public Feed()
    {
        intakeTL = new CANSparkMax(Constants.MechConstants.topLeftIntakeSparkId, MotorType.kBrushless);
        intakeTR = new CANSparkMax(Constants.MechConstants.topRightIntakeSparkId, MotorType.kBrushless);
        intakeBL = new CANSparkMax(Constants.MechConstants.bottomLeftIntakeSparkID, MotorType.kBrushless);
        intakeBR = new CANSparkMax(Constants.MechConstants.bottomRightIntakeSparkId, MotorType.kBrushless);


        intakeBL.setInverted(true);
        intakeBL.follow(intakeBR);

        intakeTL.setInverted(true);
        intakeTL.follow(intakeTR);
        
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

        intakeTL.;
        intakeBL.set(speed);
        
        

    //    feedl.set(TalonFXControlMode.PercentOutput,speed);
    //    feedr.set(TalonFXControlMode.PercentOutput,-speed);
    //    //feed.set(speed);
    }
}


