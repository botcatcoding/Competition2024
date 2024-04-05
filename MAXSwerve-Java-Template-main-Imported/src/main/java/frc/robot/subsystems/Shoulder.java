package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
    TalonFXConfiguration configShoulder = new TalonFXConfiguration();
    MotionMagicConfigs shoulderMotionMagic = configShoulder.MotionMagic;

    // ProfiledPIDController pidController;

    public Shoulder() {
        shoulderL = new TalonFX(Constants.MechConstants.shoulderLid);
        shoulderR = new TalonFX(Constants.MechConstants.shoulderRid);
        double shoulderP = 10;
        double shoulderI = 2;
        double shoulderD = 0;

        // double accel = 8;
        // double velocity = 4;
        // Constraints constraints = new Constraints(velocity, accel);
        // pidController = new ProfiledPIDController(shoulderP, 0, shoulderD,
        // constraints);
        configShoulder.Feedback.SensorToMechanismRatio = Constants.MechConstants.falconToAbsEncoderShoulder;
        configShoulder.Slot0.kP = shoulderP;
        configShoulder.Slot0.kI = shoulderI;
        configShoulder.Slot0.kD = shoulderD;
        configShoulder.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configShoulder.MotionMagic.MotionMagicCruiseVelocity = 4;
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
        shoulderL.setNeutralMode(NeutralModeValue.Brake);
        shoulderR.setNeutralMode(NeutralModeValue.Brake);
        shoulderL.setControl(new Follower(shoulderR.getDeviceID(), true));
        resetTimer.start();
    }

    public void applyConfigurationClimbSlow() {
        shoulderMotionMagic.MotionMagicCruiseVelocity = 1;
        shoulderMotionMagic.MotionMagicAcceleration = 1;
        shoulderR.getConfigurator().apply(shoulderMotionMagic);
    }

    public void applyConfigurationRegular() {
        shoulderMotionMagic.MotionMagicCruiseVelocity = 4;
        shoulderMotionMagic.MotionMagicAcceleration = 4;
        shoulderR.getConfigurator().apply(shoulderMotionMagic);
    }

    public void setBrake(boolean brake) {
        if (brake) {
            shoulderL.setNeutralMode(NeutralModeValue.Brake);
            shoulderR.setNeutralMode(NeutralModeValue.Brake);
        } else {
            shoulderL.setNeutralMode(NeutralModeValue.Coast);
            shoulderR.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    Timer resetTimer = new Timer();

    @Override
    public void periodic() {
        if (Math.abs(shoulderR.getVelocity().refresh().getValue()) < 0.01 && resetTimer.get() > .5) {
            resetTimer.reset();
            // shoulderR.setPosition(getShoulderAngle());
            // System.out.println("SET SHOULDER");
        }
        // shoulderR.setPosition(getShoulderAngle());
        SmartDashboard.putNumber("Shoulder Encoder", getShoulderAngle());
        SmartDashboard.putNumber("Shoulder Absolute", shoulderEncoder.getAbsolutePosition());
    }

    public boolean isPostioned(double shoulderRotations, double deadband) {
        boolean shoulderPositioned = Math.abs(shoulderRotations
                - shoulderR.getPosition().refresh().getValue()) < deadband;
        SmartDashboard.putString("ShoulderPositioned",
                shoulderPositioned + ":" + (Math.abs(shoulderR.getVelocity().refresh().getValue()) < .2));
        // return shoulderPositioned &&
        // Math.abs(shoulderR.getVelocity().refresh().getValue()) < .2;
        return shoulderPositioned;
    }

    public void stop() {
        shoulderR.setControl(zeroSpeedControl);
    }

    public double desiredPosition = -1;

    public void setPosition(double position) {
        desiredPosition = position;
        // double output = pidController.calculate(getShoulderAngle(), position);
        motionPos.Position = position;// (Constants.MechConstants.shoulderMiddle - position)+
                                      // Constants.MechConstants.shoulderMiddle;
        // SmartDashboard.putNumber("shoulderCommand", output);
        SmartDashboard.putNumber("desiredPosition", motionPos.Position);
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
