package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SlurpCommand;

public class Slurp extends SubsystemBase {
    CANSparkMax intakeTL;
    CANSparkMax intakeTR;
    CANSparkMax intakeBL;
    CANSparkMax intakeBR;

    private SparkPIDController intakeTRPID;
    private SparkPIDController intakeBRPID;

    public Slurp() {

        intakeTL = new CANSparkMax(Constants.MechConstants.topLeftIntakeSparkId, MotorType.kBrushless);
        intakeTR = new CANSparkMax(Constants.MechConstants.topRightIntakeSparkId, MotorType.kBrushless);
        intakeBL = new CANSparkMax(Constants.MechConstants.bottomLeftIntakeSparkID, MotorType.kBrushless);
        intakeBR = new CANSparkMax(Constants.MechConstants.bottomRightIntakeSparkId, MotorType.kBrushless);

        intakeTL.setIdleMode(IdleMode.kBrake);
        intakeTR.setIdleMode(IdleMode.kBrake);
        intakeBL.setIdleMode(IdleMode.kBrake);
        intakeBR.setIdleMode(IdleMode.kBrake);

        intakeBL.setInverted(true);
        intakeBL.follow(intakeBR);

        intakeTL.setInverted(true);
        intakeTL.follow(intakeTR);

        // feedl = new TalonFX(12);
        // feedr = new TalonFX(13);
        // feedl.setNeutralMode(NeutralMode.Coast);
        // feedr.setNeutralMode(NeutralMode.Coast);
    }

    public void setFeedVelo(double speed) {
        intakeTRPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        intakeBRPID.setReference(-speed, CANSparkMax.ControlType.kVelocity);
    }

    public void setFeedPercent(double speed) {

        // intakeTL.se;
        intakeBR.set(-speed);
        intakeTR.set(speed);
        // feedl.set(TalonFXControlMode.PercentOutput,speed);
        // feedr.set(TalonFXControlMode.PercentOutput,-speed);
        // //feed.set(speed);
    }

    public static Command slurp(Slurp slurp, DigitalInput slurpDectect) {
        return new SlurpCommand(1, false, true, slurp, slurpDectect);
    }

    public static Command slurpStop(Slurp slurp, DigitalInput slurpDectect) {
        return new SlurpCommand(0, false, false, slurp, slurpDectect);
    }

}
