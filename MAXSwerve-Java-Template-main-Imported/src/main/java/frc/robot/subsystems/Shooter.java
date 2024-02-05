package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;

public class Shooter extends SubsystemBase {

    DutyCycleOut zeroSpeedControl = new DutyCycleOut(0);
    DutyCycleOut percentOutputControl = new DutyCycleOut(0);
    // VelocityDutyCycle shooterVelocityControl = new VelocityDutyCycle(0);

    TalonFX shootTL;
    TalonFX shootTR;
    TalonFX shootBL;
    TalonFX shootBR;

    public Shooter() {
        shootTL = new TalonFX(Constants.MechConstants.shootTL);
        shootTR = new TalonFX(Constants.MechConstants.shootTR);
        shootBL = new TalonFX(Constants.MechConstants.shootBL);
        shootBR = new TalonFX(Constants.MechConstants.shootBR);

        shootTL.getConfigurator().apply(new TalonFXConfiguration());
        shootTR.getConfigurator().apply(new TalonFXConfiguration());
        shootBL.getConfigurator().apply(new TalonFXConfiguration());
        shootBR.getConfigurator().apply(new TalonFXConfiguration());

        shootTL.setNeutralMode(NeutralModeValue.Coast);
        shootTR.setNeutralMode(NeutralModeValue.Coast);
        shootBL.setNeutralMode(NeutralModeValue.Coast);
        shootBR.setNeutralMode(NeutralModeValue.Coast);

        shootTR.setControl(new Follower(shootTL.getDeviceID(), true));
        shootBR.setControl(new Follower(shootTL.getDeviceID(), true));
        shootBL.setControl(new Follower(shootTL.getDeviceID(), false));

    }

    public void setShooter(double speed) {
        percentOutputControl.Output = speed;
        shootTL.setControl(percentOutputControl);
    }

    public static Command shoot(Shooter shoot) {
        return new ShootCommand(1, shoot);
    }

    public static Command stopshoot(Shooter shoot) {
        return new ShootCommand(0, shoot);
    }

}
