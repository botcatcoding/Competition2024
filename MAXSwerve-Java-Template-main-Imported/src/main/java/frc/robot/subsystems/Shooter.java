package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    VelocityDutyCycle shooterVelocityControl = new VelocityDutyCycle(0);
  

    TalonFX shootTL;
    TalonFX shootTR;
    TalonFX shootBL;
    TalonFX shootBR;
    public Shooter()
    {
       shootTL = new TalonFX(Constants.MechConstants.shootTL);
       shootTR = new TalonFX(Constants.MechConstants.shootTR);
       shootBL = new TalonFX(Constants.MechConstants.shootBL);
       shootBR = new TalonFX(Constants.MechConstants.shootBR);
    
       shootTL.setNeutralMode(NeutralModeValue.Coast);
       shootTR.setNeutralMode(NeutralModeValue.Coast);
       shootBL.setNeutralMode(NeutralModeValue.Coast);
       shootBR.setNeutralMode(NeutralModeValue.Coast);

       
    

       shootTR.setControl(new Follower(shootTL.getDeviceID(),true));
       shootBR.setControl(new Follower(shootTL.getDeviceID(),true));
       shootBL.setControl(new Follower(shootTL.getDeviceID(),false));
       

    }

    public void setShooter(double speed)
    {
        shootTL.setControl(shooterVelocityControl);
       
        // shoot.set(speed);
    }
}
