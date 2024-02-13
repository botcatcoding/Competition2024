package frc.robot.subsystems;

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

}
