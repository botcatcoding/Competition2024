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

public class Elbow extends SubsystemBase {
    TalonFX elbowL;
    TalonFX elbowR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle posControl = new MotionMagicDutyCycle(0);
    // DutyCycleOut pidSPeedControl = new DutyCycleOut(0);
    DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(1);

    double currentAngle = 0;
    // ProfiledPIDController pidController;

    public Elbow() {
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);

        double elbowP = 6;
        double elbowI = 1;
        double elbowD = 0;

        // double accel = 8;
        // double velocity = 4;
        // Constraints constraints = new Constraints(velocity, accel);
        // pidController = new ProfiledPIDController(elbowP, elbowI, elbowD,
        // constraints);
        TalonFXConfiguration configElbow = new TalonFXConfiguration();
        configElbow.Feedback.SensorToMechanismRatio = Constants.MechConstants.falconToAbsEncoderElbow;
        configElbow.Slot0.kP = elbowP;
        configElbow.Slot0.kI = elbowI;
        configElbow.Slot0.kD = elbowD;
        configElbow.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configElbow.MotorOutput.PeakForwardDutyCycle = 1;
        configElbow.MotorOutput.PeakReverseDutyCycle = -1;
        configElbow.MotionMagic.MotionMagicCruiseVelocity = 4;
        configElbow.MotionMagic.MotionMagicAcceleration = 8;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .38;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configElbow.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.38;

        elbowL.getConfigurator().apply(configElbow);
        elbowR.getConfigurator().apply(new TalonFXConfiguration());

        // pidController.reset(getElbowAngle());
        elbowL.setPosition(getElbowAngle());
        elbowL.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setNeutralMode(NeutralModeValue.Brake);
        elbowR.setControl(new Follower(elbowL.getDeviceID(), true));

    }

    public void periodic() {
        SmartDashboard.putNumber("Elbow Encoder", getElbowAngle());
        SmartDashboard.putNumber("Elbow Absolute", elbowEncoder.getAbsolutePosition());
        // elbowL.setPosition(getElbowAngle());
    }

    public boolean isPostioned(double elbowRotations, double deadband) {
        boolean elbowPositioned = Math.abs(
                elbowRotations - elbowL.getPosition().refresh().getValue()) < deadband;

        return elbowPositioned;

    }

    public void stop() {
        elbowL.setControl(zeroSpeedControl);
    }

    boolean firstTimeCall = false;

    public void setPosition(double position) {
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

    public double getElbowAngle() {
        double absPos = elbowEncoder.getAbsolutePosition();
        if (absPos < .3) {
            absPos = absPos + 1;
        }
        double absolutePosition = ((1 - absPos)
                + Constants.MechConstants.elbowEncoderZeroValue);
        // if (absolutePosition > 1) {
        // absolutePosition = absolutePosition - 1;
        // }
        // absolutePosition = SwerveUtils.WrapAngle(absolutePosition) / 2 / Math.PI;
        return absolutePosition;

    }
}
