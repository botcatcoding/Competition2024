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
    CANSparkMax intakeT;
    CANSparkMax intakeB;

    private SparkPIDController intakeTRPID;
    private SparkPIDController intakeBRPID;

    public Slurp() {

        intakeT = new CANSparkMax(Constants.MechConstants.topIntakeSparkId, MotorType.kBrushless);
        intakeB = new CANSparkMax(Constants.MechConstants.bottomIntakeSparkId, MotorType.kBrushless);

        intakeT.restoreFactoryDefaults();
        intakeB.restoreFactoryDefaults();

        intakeT.setSmartCurrentLimit(40);
        intakeB.setSmartCurrentLimit(40);

        double p = 0.00005;
        double i = 0.000001;
        double d = 0.0001;

        intakeT.getPIDController().setP(p);
        intakeB.getPIDController().setP(p);

        intakeT.getPIDController().setI(i);
        intakeB.getPIDController().setI(i);

        intakeT.getPIDController().setD(d);
        intakeB.getPIDController().setD(d);

        intakeT.setIdleMode(IdleMode.kBrake);
        intakeB.setIdleMode(IdleMode.kBrake);

        intakeTRPID = intakeT.getPIDController();
        intakeBRPID = intakeB.getPIDController();

        intakeT.setInverted(true);

        intakeT.burnFlash();
        intakeB.burnFlash();
        // 0.00009999999747378752 kpid p
        // 9.99999993922529e-9 kpid i
        // feedl = new TalonFX(12);
        // feedr = new TalonFX(13);
        // feedl.setNeutralMode(NeutralMode.Coast);
        // feedr.setNeutralMode(NeutralMode.Coast);
    }

    public void setFeedVelo(double speed) {
        intakeTRPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        intakeBRPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public void setFeedPercent(double speed) {

        // intakeTL.se;
        intakeB.set(speed);
        intakeT.set(speed);
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
        int error = (int) ((360 - dpad) - yaw);
        error = (error % 360) + (error < 0 ? 360 : 0);
        if (error > 0 && error < 90) {
            return true;
        } else if (error > 90 && error < 270) {
            return false;
        }
        return true;
    }

}
