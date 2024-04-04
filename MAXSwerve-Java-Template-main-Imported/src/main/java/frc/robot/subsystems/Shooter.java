package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    DutyCycleOut percentOutputControlL = new DutyCycleOut(0);
    DutyCycleOut percentOutputControlR = new DutyCycleOut(0);
    VelocityDutyCycle shooterVelocityControlL = new VelocityDutyCycle(100);
    VelocityDutyCycle shooterVelocityControlR = new VelocityDutyCycle(100);

    TalonFX shootTL;
    TalonFX shootTR;
    TalonFX shootBL;
    TalonFX shootBR;

    public Shooter() {

        double shooterP = .02;
        double shooterI = 0.3;
        double shooterD = 0;

        TalonFXConfiguration configShooter = new TalonFXConfiguration();

        configShooter.Slot0.kP = shooterP;
        configShooter.Slot0.kI = shooterI;
        configShooter.Slot0.kD = shooterD;

        shootTL = new TalonFX(Constants.MechConstants.shootTL);
        shootTR = new TalonFX(Constants.MechConstants.shootTR);
        shootBL = new TalonFX(Constants.MechConstants.shootBL);
        shootBR = new TalonFX(Constants.MechConstants.shootBR);

        shootTL.getConfigurator().apply(configShooter);
        shootTR.getConfigurator().apply(configShooter);
        shootBL.getConfigurator().apply(configShooter);
        shootBR.getConfigurator().apply(configShooter);

        shootTL.setNeutralMode(NeutralModeValue.Coast);
        shootTR.setNeutralMode(NeutralModeValue.Coast);
        shootBL.setNeutralMode(NeutralModeValue.Coast);
        shootBR.setNeutralMode(NeutralModeValue.Coast);

        shootBR.setControl(new Follower(shootTR.getDeviceID(), true));
        shootBL.setControl(new Follower(shootTL.getDeviceID(), true));
        // shootBL.setControl(new Follower(shootTL.getDeviceID(), false));

    }

    double targetVelocity;

    public void setShooterVelocity(double velocityL, double velocityR) {
        SmartDashboard.putNumber("ShootLeft", velocityL);
        SmartDashboard.putNumber("ShootRight", velocityR);
        targetVelocity = -velocityL;
        // percentOutputControlL.Output = speedl;
        // percentOutputControlR.Output = speedr;
        shooterVelocityControlL.Velocity = -velocityL;
        shooterVelocityControlR.Velocity = velocityR;
        shootTL.setControl(shooterVelocityControlL);
        // shooterVelocityControlR.Velocity = 0;
        shootTR.setControl(shooterVelocityControlR);

    }

    public void setShooterPercent(double speed) {
        shootTL.set(speed);
        shootTR.set(speed);
    }

    public boolean isUpToSpeed() {
        // SmartDashboard.putNumber("velocity",
        // shootTL.getVelocity().refresh().getValue());
        // SmartDashboard.putNumber("targetVelocity", targetVelocity);
        return Math.abs(shootTL.getVelocity().refresh().getValue() - targetVelocity) < .05;

    }
    // public static Command shoot(Shooter shoot) {
    // return new ShootCommand(shoot, 100, true);
    // }

    // public static Command stopshoot(Shooter shoot) {
    // return new ShootCommand(shoot, 0, false);
    // }
    double maxSpeed = -100;
    Timer speedreset = new Timer();

    @Override
    public void periodic() {
        // speedreset.start();
        // if (speedreset.get() > 5) {
        // speedreset.reset();
        // maxSpeed = -100;
        // // System.out.println("reset");
        // } else {
        // SmartDashboard.putNumber("max speed", maxSpeed);
        // }
        // if (maxSpeed < shootTL.getVelocity().refresh().getValue()) {
        // maxSpeed = shootTL.getVelocity().refresh().getValue();
        // }

    }
}
