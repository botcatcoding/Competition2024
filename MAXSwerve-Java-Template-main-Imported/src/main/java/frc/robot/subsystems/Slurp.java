package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    // TalonFX feedl;
    // TalonFX feedr;
    public Slurp() {
        intakeTL = new CANSparkMax(Constants.MechConstants.topLeftIntakeSparkId, MotorType.kBrushless);
        intakeTR = new CANSparkMax(Constants.MechConstants.topRightIntakeSparkId, MotorType.kBrushless);
        intakeBL = new CANSparkMax(Constants.MechConstants.bottomLeftIntakeSparkID, MotorType.kBrushless);
        intakeBR = new CANSparkMax(Constants.MechConstants.bottomRightIntakeSparkId, MotorType.kBrushless);

        intakeBL.setInverted(true);
        intakeBL.follow(intakeBR);

        intakeTL.setInverted(true);
        intakeTL.follow(intakeTR);

        // feedl = new TalonFX(12);
        // feedr = new TalonFX(13);
        // feedl.setNeutralMode(NeutralMode.Coast);
        // feedr.setNeutralMode(NeutralMode.Coast);
    }

    public void setFeedVelo(int ticks) {
        intakeTRPID.setReference(ticks, CANSparkMax.ControlType.kVelocity);
        intakeBRPID.setReference(-ticks, CANSparkMax.ControlType.kVelocity);
    }

    public void setFeedPercent(double speed) {

        // intakeTL.se;
        intakeBR.set(-speed);
        intakeTR.set(speed);
        // feedl.set(TalonFXControlMode.PercentOutput,speed);
        // feedr.set(TalonFXControlMode.PercentOutput,-speed);
        // //feed.set(speed);
    }

    public static Command slurp(Slurp slurp) {
        return new SlurpCommand(500, slurp);
    }

}
