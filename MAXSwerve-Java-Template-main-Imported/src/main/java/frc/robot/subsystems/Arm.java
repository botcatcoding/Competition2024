package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetElbowAndShoulderByPositionCommand;

public class Arm extends SubsystemBase {
    TalonFX shoulderL;
    TalonFX shoulderR;
    TalonFX elbowL;
    TalonFX elbowR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle shoulderMotionControl = new MotionMagicDutyCycle(0);
    MotionMagicDutyCycle elbowMotionControl = new MotionMagicDutyCycle(0);

    DutyCycleEncoder shoulEncoder = new DutyCycleEncoder(0);

    public Arm() {
        shoulderL = new TalonFX(Constants.MechConstants.shoulderLid);
        shoulderR = new TalonFX(Constants.MechConstants.shoulderRid);
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);
        // read absolute encoder and convert encoder to falcon
        double myValue = (shoulEncoder.getAbsolutePosition() - Constants.MechConstants.shoulderEncoderZeroValue)
                * Constants.MechConstants.falconToAbsEncoderShoulder;

        shoulderL.setNeutralMode(NeutralModeValue.Brake);
        shoulderR.setNeutralMode(NeutralModeValue.Brake);
        shoulderR.setControl(new Follower(shoulderL.getDeviceID(), false));

        elbowL.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setControl(new Follower(elbowL.getDeviceID(), false));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder", shoulEncoder.getAbsolutePosition());
    }

    public void setPosition(double shoulderDegrees, double elbowDegrees) {
        double shoulderRotations = shoulderDegrees * Constants.MechConstants.shoulderRotationToDegrees;
        shoulderMotionControl.Position = shoulderRotations;
        shoulderL.setControl(shoulderMotionControl);
        // shoulderL.set(ControlMode.MotionMagic, shoulderTicks);

        double elbowRotations = elbowDegrees * Constants.MechConstants.elbowRotationToDegrees;
        elbowMotionControl.Position = elbowRotations;
        elbowL.setControl(elbowMotionControl);
        // elbowL.set(ControlMode.MotionMagic, elbowTicks);
    }

    public void stop() {
        shoulderL.setControl(zeroSpeedControl);
        elbowL.setControl(zeroSpeedControl);
    }

    public boolean isPostioned(double shoulderDegrees, double elbowDegrees) {
        double shoulderRotation = shoulderDegrees * Constants.MechConstants.shoulderRotationToDegrees;
        boolean shoulderPositioned = Math.abs(shoulderRotation
                - shoulderL.getClosedLoopReference().getValue()) < Constants.MechConstants.shoulderDeadband;

        double elbowRotation = elbowDegrees * Constants.MechConstants.elbowRotationToDegrees;
        boolean elbowPositioned = Math.abs(
                elbowRotation - elbowL.getClosedLoopReference().getValue()) < Constants.MechConstants.elbowDeadband;

        return shoulderPositioned && elbowPositioned;

    }

    public static Command gotoSlurpPosition(Arm arm) {
        return new SetElbowAndShoulderByPositionCommand(80, 140, arm);
    }

}
