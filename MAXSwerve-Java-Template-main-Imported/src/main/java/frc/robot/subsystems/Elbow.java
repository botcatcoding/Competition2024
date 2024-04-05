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

public class Elbow extends SubsystemBase {
    TalonFX elbowL;
    TalonFX elbowR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle posControl = new MotionMagicDutyCycle(0);
    DutyCycleOut simplePerc = new DutyCycleOut(0);
    DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(1);

    double currentAngle = 0;
    TalonFXConfiguration configElbow = new TalonFXConfiguration();
    MotionMagicConfigs elbowMotionMagic = configElbow.MotionMagic;
    // ProfiledPIDController pidController;

    public Elbow() {
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);

        double elbowFF = .3;
        double elbowP = 5;
        // double elbowI = 8;
        // double elbowD = .5;

        double elbowI = 1;
        double elbowD = 0;
        // double accel = 8;
        // double velocity = 4;
        // Constraints constraints = new Constraints(velocity, accel);
        // pidController = new ProfiledPIDController(elbowP, elbowI, elbowD,
        // constraints);
        configElbow.Audio.BeepOnConfig = false;
        // configElbow.Audio.BeepOnBoot = false;
        configElbow.Feedback.SensorToMechanismRatio = Constants.MechConstants.falconToAbsEncoderElbow;
        configElbow.Slot0.kP = elbowP;
        configElbow.Slot0.kI = elbowI;
        configElbow.Slot0.kD = elbowD;
        configElbow.Slot0.kV = elbowFF;
        configElbow.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configElbow.MotorOutput.PeakForwardDutyCycle = 1;
        configElbow.MotorOutput.PeakReverseDutyCycle = -1;
        configElbow.MotionMagic.MotionMagicCruiseVelocity = 4;// dont forget to change this below
        configElbow.MotionMagic.MotionMagicAcceleration = 8;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .41;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.41;

        elbowL.getConfigurator().apply(configElbow);
        elbowR.getConfigurator().apply(new TalonFXConfiguration());

        // pidController.reset(getElbowAngle());
        elbowL.setPosition(getElbowAngle());
        elbowL.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setControl(new Follower(elbowL.getDeviceID(), true));
        resetTimer.start();
    }

    public void setBrake(boolean brake) {
        if (brake) {
            elbowL.setNeutralMode(NeutralModeValue.Brake);
            elbowR.setNeutralMode(NeutralModeValue.Brake);
        } else {
            elbowL.setNeutralMode(NeutralModeValue.Coast);
            elbowR.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    public void applyConfigurationClimbSlow() {
        elbowMotionMagic.MotionMagicCruiseVelocity = 2;
        elbowMotionMagic.MotionMagicAcceleration = 1;
        elbowL.getConfigurator().apply(elbowMotionMagic);
    }

    public void applyConfigurationRegular() {
        elbowMotionMagic.MotionMagicCruiseVelocity = 4;
        elbowMotionMagic.MotionMagicAcceleration = 8;
        elbowL.getConfigurator().apply(elbowMotionMagic);
    }

    Timer resetTimer = new Timer();

    public void periodic() {
        if (Math.abs(elbowL.getVelocity().refresh().getValue()) < 0.01 && resetTimer.get() > .2
                && !encoderUnlreliable) {
            resetTimer.reset();
            elbowL.setPosition(getElbowAngle());
            // System.out.println("SET ELBOW");
        }

        SmartDashboard.putBoolean("Elbow Reliable", !encoderUnlreliable);
        SmartDashboard.putNumber("Elbow Encoder", getElbowAngle());
        SmartDashboard.putNumber("Elbow Absolute", elbowEncoder.getAbsolutePosition());
        // elbowL.setPosition(getElbowAngle());
    }

    public void setPerc(double perc) {
        simplePerc.Output = perc;
        elbowL.setControl(simplePerc);
    }

    public boolean isPostioned(double elbowRotations, double deadband) {
        boolean elbowPositioned = Math.abs(
                elbowRotations - elbowL.getPosition().refresh().getValue()) < deadband;

        SmartDashboard.putString("ElbowPosotioned",
                elbowPositioned + ":" + (Math.abs(elbowL.getVelocity().refresh().getValue()) < .2));
        // return elbowPositioned && Math.abs(elbowR.getVelocity().refresh().getValue())
        // < .2;
        return elbowPositioned;

    }

    public void stop() {
        elbowL.setControl(zeroSpeedControl);
    }

    boolean firstTimeCall = false;
    public double desiredPosition = -1;

    public void setPosition(double position) {
        desiredPosition = position;
        SmartDashboard.putNumber("Elbow Desired", position);
        firstTimeCall = true;
        currentAngle = position;
        posControl.Position = position;
        elbowL.setControl(posControl);
        // double output = pidController.calculate(getElbowAngle(), position);
        // pidSPeedControl.Output = output;
        // elbowL.setControl(pidSPeedControl);
    }

    public void dontMove() {
        // if (firstTimeCall) {
        // currentAngle = getElbowAngle();
        // }
        // firstTimeCall = false;
        // double output = pidController.calculate(getElbowAngle() - currentAngle);
        // pidSPeedControl.Output = output;
        elbowL.setControl(zeroSpeedControl);
    }

    boolean encoderUnlreliable = false;

    public double getElbowAngle() {
        double absPos = elbowEncoder.getAbsolutePosition();
        if (Math.abs(absPos - 1) < 0.01) {
            encoderUnlreliable = true;
        } else {
            encoderUnlreliable = false;
        }
        SmartDashboard.putNumber("ElbowAbsolute", absPos);
        double absolutePosition = ((1 - absPos)
                + Constants.MechConstants.elbowEncoderZeroValue) + .5;
        absolutePosition = SwerveUtils.WrapAngle(absolutePosition * 2 * Math.PI) / 2 / Math.PI - .5;
        return absolutePosition;

    }
}
