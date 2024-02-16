package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetElbowAndShoulderByPositionCommand;
import frc.utils.SwerveUtils;

public class Arm extends SubsystemBase {
    TalonFX shoulderL;
    TalonFX shoulderR;
    TalonFX elbowL;
    TalonFX elbowR;

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    MotionMagicDutyCycle shoulderMotionControl = new MotionMagicDutyCycle(0);
    MotionMagicDutyCycle elbowMotionControl = new MotionMagicDutyCycle(0);

    DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(0);
    DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(1);

    public Arm() {
        shoulderL = new TalonFX(Constants.MechConstants.shoulderLid);
        shoulderR = new TalonFX(Constants.MechConstants.shoulderRid);
        elbowL = new TalonFX(Constants.MechConstants.elbowLid);
        elbowR = new TalonFX(Constants.MechConstants.elbowRid);

        TalonFXConfiguration configShoulder = new TalonFXConfiguration();
        configShoulder.Feedback.SensorToMechanismRatio = -Constants.MechConstants.falconToAbsEncoderShoulder;

        TalonFXConfiguration configElbow = new TalonFXConfiguration();
        configElbow.Feedback.SensorToMechanismRatio = -Constants.MechConstants.falconToAbsEncoderElbow;

        shoulderL.getConfigurator().apply(configShoulder);
        shoulderR.getConfigurator().apply(new TalonFXConfiguration());
        elbowL.getConfigurator().apply(configElbow);
        elbowR.getConfigurator().apply(new TalonFXConfiguration());
        // read absolute encoder and convert encoder to falcon
        // double myValue = (shoulderEncoder.getAbsolutePosition() -
        // Constants.MechConstants.shoulderEncoderZeroValue)
        // * Constants.MechConstants.falconToAbsEncoderShoulder;
        shoulderL.setPosition(getShoulderAngle());
        shoulderL.setNeutralMode(NeutralModeValue.Coast);
        shoulderR.setNeutralMode(NeutralModeValue.Coast);
        shoulderR.setControl(new Follower(shoulderL.getDeviceID(), false));

        elbowL.setPosition(getElbowAngle());
        elbowL.setNeutralMode(NeutralModeValue.Coast);
        elbowR.setNeutralMode(NeutralModeValue.Coast);
        elbowR.setControl(new Follower(elbowL.getDeviceID(), false));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder", getShoulderAngle());
        SmartDashboard.putNumber("Elbow Encoder", elbowEncoder.getAbsolutePosition());
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

    public static KinematicsResult calculateArmKinematics(double tx, double ty, double tz, double bx, double by,
            double bz, double sa) {
        double differenceY = ty - by;
        double differenceX = tx - bx;
        double differenceZ = tz - bz;
        double yaw = Math.asin(differenceY / differenceX);
        double shoulderZ = bz + Constants.MechConstants.kShoulderZOffset;
        double d = Math.cos(sa) / Constants.MechConstants.kShoulderLength;
        double l = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));
        double e = l - d;
        double f = Math.sin(sa) * Constants.MechConstants.kShoulderLength + shoulderZ;
        double g = differenceZ - f;
        // System.out.println(e+"\t"+g);
        double elbowAngle = Rotation2d.fromDegrees(180).getRadians() - sa + Math.atan2(g, e);

        return new KinematicsResult(yaw, elbowAngle);
    }

    public static class KinematicsResult {
        public double yaw;
        public double elbowAngle;

        public KinematicsResult(double y, double e) {
            yaw = y;
            elbowAngle = e;
        }

    }

    public double getShoulderAngle() {
        double absolutePosition = ((1 - shoulderEncoder.getAbsolutePosition())
                + Constants.MechConstants.shoulderEncoderZeroValue) * 2 * Math.PI;
        absolutePosition = SwerveUtils.WrapAngle(absolutePosition) / 2 / Math.PI;
        if (absolutePosition > .7) {
            absolutePosition = 0;
        }
        return absolutePosition;

    }

    public double getElbowAngle() {
        double absolutePosition = ((1 - elbowEncoder.getAbsolutePosition())
                + Constants.MechConstants.elbowEncoderZeroValue) * 2 * Math.PI;
        absolutePosition = SwerveUtils.WrapAngle(absolutePosition) / 2 / Math.PI;
        return absolutePosition;

    }
}
