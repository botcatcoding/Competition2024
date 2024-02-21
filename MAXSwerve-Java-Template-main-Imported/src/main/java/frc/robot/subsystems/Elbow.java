package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {
    TalonFX elbowL;
    TalonFX elbowR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle elbowMotionControl = new MotionMagicDutyCycle(0);
    DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(1);
    double currentAngle = 0;

    public Elbow() {
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);

        double elbowP = 5;
        double elbowD = 0;

        TalonFXConfiguration configElbow = new TalonFXConfiguration();
        configElbow.Feedback.SensorToMechanismRatio = Constants.MechConstants.falconToAbsEncoderElbow;
        configElbow.Slot0.kP = elbowP;
        configElbow.Slot0.kD = elbowD;
        configElbow.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configElbow.MotorOutput.PeakForwardDutyCycle = 1;
        configElbow.MotorOutput.PeakReverseDutyCycle = -1;
        configElbow.MotionMagic.MotionMagicCruiseVelocity = 1;
        configElbow.MotionMagic.MotionMagicAcceleration = 8;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .4;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.4;

        elbowL.getConfigurator().apply(configElbow);
        elbowR.getConfigurator().apply(new TalonFXConfiguration());

        elbowL.setPosition(getElbowAngle());
        elbowL.setNeutralMode(NeutralModeValue.Coast);
        elbowR.setNeutralMode(NeutralModeValue.Coast);
        elbowR.setControl(new Follower(elbowL.getDeviceID(), true));

    }

    public void periodic() {
        SmartDashboard.putNumber("Elbow Encoder", getElbowAngle());
        // elbowL.setPosition(getElbowAngle());
    }

    public boolean isPostioned(double elbowRotations, double deadband) {
        boolean elbowPositioned = Math.abs(
                elbowRotations - elbowL.getClosedLoopReference().getValue()) < deadband;

        return elbowPositioned;

    }

    public void stop() {
        elbowL.setControl(zeroSpeedControl);
    }

    public void setPosition(double position) {
        currentAngle = position;
        elbowMotionControl.Position = position;
        elbowL.setControl(elbowMotionControl);
    }

    public void dontMove() {
        elbowMotionControl.Position = currentAngle;
        elbowL.setControl(elbowMotionControl);
    }

    public double getElbowAngle() {
        double absPos = elbowEncoder.getAbsolutePosition();
        if (absPos < .25) {
            absPos = absPos + 1;

        }
        SmartDashboard.putNumber("absPos", absPos);
        double absolutePosition = ((1 - absPos)
                + Constants.MechConstants.elbowEncoderZeroValue);
        // if (absolutePosition < .5) {
        // absolutePosition = absolutePosition - 1;
        // }
        // absolutePosition = SwerveUtils.WrapAngle(absolutePosition) / 2 / Math.PI;
        return absolutePosition;

    }
}
