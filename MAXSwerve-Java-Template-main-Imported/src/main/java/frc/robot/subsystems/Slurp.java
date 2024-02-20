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

        intakeTL.restoreFactoryDefaults();
        intakeTR.restoreFactoryDefaults();
        intakeBL.restoreFactoryDefaults();
        intakeBR.restoreFactoryDefaults();

        intakeTL.setSmartCurrentLimit(40);
        intakeTR.setSmartCurrentLimit(40);
        intakeBL.setSmartCurrentLimit(40);
        intakeBR.setSmartCurrentLimit(40);

        double p = 0.00009999999747378752;
        double i = 9.99999993922529e-9;
        double d = 0;

        intakeTL.getPIDController().setP(p);
        intakeTR.getPIDController().setP(p);
        intakeBL.getPIDController().setP(p);
        intakeBR.getPIDController().setP(p);

        intakeTL.getPIDController().setI(i);
        intakeTR.getPIDController().setI(i);
        intakeBL.getPIDController().setI(i);
        intakeBR.getPIDController().setI(i);

        intakeTL.getPIDController().setD(d);
        intakeTR.getPIDController().setD(d);
        intakeBL.getPIDController().setD(d);
        intakeBR.getPIDController().setD(d);

        intakeTL.setIdleMode(IdleMode.kBrake);
        intakeTR.setIdleMode(IdleMode.kBrake);
        intakeBL.setIdleMode(IdleMode.kBrake);
        intakeBR.setIdleMode(IdleMode.kBrake);

        intakeBL.setInverted(true);
        intakeBL.follow(intakeBR);

        intakeTL.setInverted(true);
        intakeTL.follow(intakeTR);

        intakeTRPID = intakeTR.getPIDController();
        intakeBRPID = intakeBR.getPIDController();

        intakeTL.burnFlash();
        intakeTR.burnFlash();
        intakeBL.burnFlash();
        intakeBR.burnFlash();
        // 0.00009999999747378752 kpid p
        // 9.99999993922529e-9 kpid i
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

    public static boolean slurpFieldOrient(int dpad, double yaw) {// dpad = 0,90,180,270. yaw = 0-360
        int error = (int) (dpad - yaw);
        error = (error % 360) + (error < 0 ? 360 : 0);
        if (error > 0 && error < 90) {
            return true;
        } else if (error > 90 && error < 270) {
            return false;
        }
        return true;
    }

}
