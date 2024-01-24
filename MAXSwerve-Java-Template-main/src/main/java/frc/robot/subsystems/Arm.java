package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    TalonFX shoulderL;
    TalonFX shoulderR;
    TalonFX elbowL;
    TalonFX elbowR;
    public Arm()
    {
        shoulderL= new TalonFX(Constants.MechConstants.shoulderLid);
        shoulderR = new TalonFX(Constants.MechConstants.shoulderRid);
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);
        

        shoulderL.setNeutralMode(NeutralMode.Brake);
        shoulderR.setNeutralMode(NeutralMode.Brake);
        shoulderR.follow(shoulderL);

        elbowL.setNeutralMode(NeutralMode.Brake);
        elbowR.setNeutralMode(NeutralMode.Brake);
        elbowR.follow(elbowL);
    }
    public void setPosition(double shoulderDegrees,double elbowDegrees)
    {
        double shoulderTicks = shoulderDegrees * Constants.MechConstants.shoulderTicksToDegrees;
        shoulderL.set(ControlMode.MotionMagic, shoulderTicks);
        
        double elbowTicks = elbowDegrees * Constants.MechConstants.elbowTicksToDegrees;
        elbowL.set(ControlMode.MotionMagic, elbowTicks);
    }

    public void stop(){
        shoulderL.set(ControlMode.PercentOutput, 0);
        elbowL.set(ControlMode.PercentOutput, 0);
    }

    public boolean isPostioned(double shoulderDegrees, double elbowDegrees){
        double shoulderTicks = shoulderDegrees * Constants.MechConstants.shoulderTicksToDegrees;
        boolean shoulderPositioned = Math.abs(shoulderTicks-shoulderL.getSelectedSensorPosition())<Constants.MechConstants.shoulderDeadband;
        
        double elbowTicks = elbowDegrees * Constants.MechConstants.elbowTicksToDegrees;
        boolean elbowPositioned = Math.abs(elbowTicks-elbowL.getSelectedSensorPosition())<Constants.MechConstants.elbowDeadband;

        return shoulderPositioned && elbowPositioned; 
        

    }


}
