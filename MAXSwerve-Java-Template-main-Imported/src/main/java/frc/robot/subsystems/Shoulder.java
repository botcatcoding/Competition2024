package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.SwerveUtils;

public class Shoulder extends SubsystemBase {
    TalonFX shoulderL;
    TalonFX shoulderR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    // DutyCycleOut pidSPeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle motionPos = new MotionMagicDutyCycle(.25);
    DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(0);

    // ProfiledPIDController pidController;

    public Shoulder() {
        shoulderL = new TalonFX(Constants.MechConstants.shoulderLid);
        shoulderR = new TalonFX(Constants.MechConstants.shoulderRid);
        double shoulderP = 8;
        double shoulderI = 1;
        double shoulderD = 0;

        // double accel = 8;
        // double velocity = 4;
        // Constraints constraints = new Constraints(velocity, accel);
        // pidController = new ProfiledPIDController(shoulderP, 0, shoulderD,
        // constraints);
        TalonFXConfiguration configShoulder = new TalonFXConfiguration();
        configShoulder.Feedback.SensorToMechanismRatio = Constants.MechConstants.falconToAbsEncoderShoulder;
        configShoulder.Slot0.kP = shoulderP;
        configShoulder.Slot0.kI = shoulderI;
        configShoulder.Slot0.kD = shoulderD;
        configShoulder.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configShoulder.MotionMagic.MotionMagicCruiseVelocity = 2;
        configShoulder.MotionMagic.MotionMagicAcceleration = 4;
        configShoulder.MotorOutput.PeakForwardDutyCycle = 1;
        configShoulder.MotorOutput.PeakReverseDutyCycle = -1;
        configShoulder.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configShoulder.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .5;
        configShoulder.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configShoulder.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .00;

        shoulderR.getConfigurator().apply(configShoulder);
        shoulderL.getConfigurator().apply(new TalonFXConfiguration());

        shoulderR.setPosition(getShoulderAngle());
        // pidController.reset(getShoulderAngle());
        shoulderL.setNeutralMode(NeutralModeValue.Coast);
        shoulderR.setNeutralMode(NeutralModeValue.Coast);
        shoulderL.setControl(new Follower(shoulderR.getDeviceID(), true));

    }

    @Override
    public void periodic() {
        // shoulderR.setPosition(getShoulderAngle());
        SmartDashboard.putNumber("Shoulder Encoder", getShoulderAngle());
        SmartDashboard.putNumber("Shoulder Absolute", shoulderEncoder.getAbsolutePosition());
    }

    public boolean isPostioned(double shoulderRotations, double deadband) {
        boolean shoulderPositioned = Math.abs(shoulderRotations
                - shoulderR.getPosition().refresh().getValue()) < deadband;
        return shoulderPositioned;
    }

    public void stop() {
        shoulderR.setControl(zeroSpeedControl);
    }

    public void setPosition(double position) {
        // double output = pidController.calculate(getShoulderAngle(), position);
        motionPos.Position = position;
        // SmartDashboard.putNumber("shoulderCommand", output);
        SmartDashboard.putNumber("shoulderPosition", getShoulderAngle());
        SmartDashboard.putNumber("desiredPosition", position);
        // shoulderMotionControl.Position = position;
        shoulderR.setControl(motionPos);
    }

    public double getShoulderAngle() {
        double absolutePosition = ((1 - shoulderEncoder.getAbsolutePosition())
                + Constants.MechConstants.shoulderEncoderZeroValue) * 2 * Math.PI;
        absolutePosition = SwerveUtils.WrapAngle(absolutePosition) / 2 / Math.PI;
        // if (absolutePosition > .7) {
        // absolutePosition = 0;
        // }
        return absolutePosition;

    }
}
