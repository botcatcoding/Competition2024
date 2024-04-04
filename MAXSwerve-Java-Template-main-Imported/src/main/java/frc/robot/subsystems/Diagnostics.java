package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map.Entry;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autos.AutoManager;

public class Diagnostics extends SubsystemBase {
    private DriveSubsystem m_robotDrive;
    private Elbow elbow;
    private Shoulder shoulder;
    private Shooter shooter;
    private Slurp slurp;
    private AutoManager am;
    private NetworkTable telemetryNetworkTable;
    private HashMap<String, CANSparkMax> sparkMotors = new HashMap<>();
    private HashMap<String, TalonFX> mechMotors = new HashMap<>();
    private HashMap<String, FloatPublisher> telemMotors = new HashMap<>();
    private FloatPublisher statusPub;
    private FloatPublisher elbowPublisher;
    private FloatPublisher shoulderPublisher;
    private FloatPublisher elbowdPublisher;
    private FloatPublisher shoulderdPublisher;
    //
    static FloatPublisher voltagePub;
    static StringPublisher autoSelection;
    static StringPublisher alliance;
    static StringSubscriber autoSelector;
    static FloatPublisher xPub;
    static FloatPublisher yPub;
    static FloatPublisher rotPub;

    static String auto;

    public Diagnostics(DriveSubsystem driveSubsystem, Elbow elbow, Shooter shooter, Shoulder shoulder, Slurp slurp,
            AutoManager am) {
        super();
        this.elbow = elbow;
        this.shoulder = shoulder;
        this.shooter = shooter;
        this.slurp = slurp;
        this.am = am;
        m_robotDrive = driveSubsystem;
        telemetryNetworkTable = NetworkTableInstance.getDefault().getTable("telem");
        init();
        timer.start();
    }

    public void publish() {
        statusPub = telemetryNetworkTable.getFloatTopic("status").publish();

        for (Entry<String, CANSparkMax> entry : sparkMotors.entrySet()) {
            FloatPublisher pub = telemetryNetworkTable.getFloatTopic(entry.getKey()).publish();
            telemMotors.put(entry.getKey(), pub);
            pub.set(-1);
        }
        for (Entry<String, TalonFX> entry : mechMotors.entrySet()) {
            FloatPublisher pub = telemetryNetworkTable.getFloatTopic(entry.getKey()).publish();
            telemMotors.put(entry.getKey(), pub);
            pub.set(-1);
        }

        shoulderPublisher = telemetryNetworkTable.getFloatTopic("shoulder").publish();
        elbowPublisher = telemetryNetworkTable.getFloatTopic("elbow").publish();
        shoulderdPublisher = telemetryNetworkTable.getFloatTopic("shoulderd").publish();
        elbowdPublisher = telemetryNetworkTable.getFloatTopic("elbowd").publish();

        voltagePub = telemetryNetworkTable.getFloatTopic("voltage").publish();
        xPub = telemetryNetworkTable.getFloatTopic("x").publish();
        yPub = telemetryNetworkTable.getFloatTopic("y").publish();
        rotPub = telemetryNetworkTable.getFloatTopic("rot").publish();
        autoSelection = telemetryNetworkTable.getStringTopic("autoSelection").publish();
        alliance = telemetryNetworkTable.getStringTopic("alliance").publish();
        autoSelector = telemetryNetworkTable.getStringTopic("autoSelector").subscribe("");

    }

    public void init() {
        initMAXSwerveModule(m_robotDrive.m_frontLeft);
        initMAXSwerveModule(m_robotDrive.m_frontRight);
        initMAXSwerveModule(m_robotDrive.m_rearLeft);
        initMAXSwerveModule(m_robotDrive.m_rearRight);
        mechMotors.put("motor" + elbow.elbowL.getDeviceID(), elbow.elbowL);
        mechMotors.put("motor" + elbow.elbowR.getDeviceID(), elbow.elbowR);
        mechMotors.put("motor" + shoulder.shoulderL.getDeviceID(), shoulder.shoulderL);
        mechMotors.put("motor" + shoulder.shoulderR.getDeviceID(), shoulder.shoulderR);
        mechMotors.put("motor" + shooter.shootBL.getDeviceID(), shooter.shootBL);
        mechMotors.put("motor" + shooter.shootTL.getDeviceID(), shooter.shootTL);
        mechMotors.put("motor" + shooter.shootBR.getDeviceID(), shooter.shootBR);
        mechMotors.put("motor" + shooter.shootTR.getDeviceID(), shooter.shootTR);
        sparkMotors.put("motor" + slurp.intakeB.getDeviceId(), slurp.intakeB);
        sparkMotors.put("motor" + slurp.intakeT.getDeviceId(), slurp.intakeT);
    }

    public void initMAXSwerveModule(MAXSwerveModule msm) {
        String driveMotor = "motor" + msm.m_drivingSparkMax.getDeviceId();
        sparkMotors.put(driveMotor, msm.m_drivingSparkMax);
        String turnMotor = "motor" + msm.m_turningSparkMax.getDeviceId();
        sparkMotors.put(turnMotor, msm.m_turningSparkMax);
    }

    Timer timer = new Timer();

    @Override
    public void periodic() {
        boolean sendTelem = DriverStation.isDisabled() && timer.get() > .1;
        sendTelem = sendTelem || DriverStation.isEnabled() && timer.get() > 1;
        if (sendTelem) {
            timer.reset();
            sendTelemetry();
            // System.out.println("SEND");
        }
        // if (true) {
        // return;
        // }
        // SmartDashboard.putNumber("test",
        // m_robotDrive.m_frontLeft.m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        // String drivetrainFaults = m_robotDrive.diagnostic();
        // if (drivetrainFaults == null) {
        // SmartDashboard.putString("drivetrainFaults", "Drivetrain Healthy");
        // } else {
        // SmartDashboard.putString("drivetrainFaults", drivetrainFaults);
        // }
    }

    void sendTelemetry() {
        // status
        if (DriverStation.isDisabled()) {
            statusPub.set(.5f);
        } else if (DriverStation.isAutonomous()) {
            statusPub.set(1.5f);
        } else if (DriverStation.isTeleop()) {
            statusPub.set(2.5f);
        }

        // all motors
        for (Entry<String, CANSparkMax> entry : sparkMotors.entrySet()) {
            CANSparkMax theMotor = entry.getValue();
            telemMotors.get(entry.getKey()).set(diagnoseSparkMax(theMotor));
        }
        for (Entry<String, TalonFX> entry : mechMotors.entrySet()) {
            TalonFX theMotor = entry.getValue();
            telemMotors.get(entry.getKey()).set(diagnoseFalcon(theMotor));
        }

        // arm kina
        elbowPublisher.set((float) elbow.getElbowAngle());
        shoulderPublisher.set((float) shoulder.getShoulderAngle());
        elbowdPublisher.set((float) elbow.desiredPosition);
        shoulderdPublisher.set((float) shoulder.desiredPosition);

        voltagePub.set((float) RobotController.getInputVoltage());
        xPub.set((float) m_robotDrive.getPose().getX());
        yPub.set((float) m_robotDrive.getPose().getY());
        rotPub.set((float) m_robotDrive.getPose().getRotation().getRadians());
        alliance.set(DriverStation.getAlliance().toString());
        auto = autoSelector.get("");
        am.setAuto(auto);
        autoSelection.set(am.getAuto());
    }

    static float diagnoseSparkMax(CANSparkMax theMotor) {
        // System.out.println(theMotor.getAbsoluteEncoder(Type.kDutyCycle).getZeroOffset());
        if (theMotor.getFirmwareVersion() == 0) {
            return .5f;
        } else {
            // if (theMotor.getFault(FaultID.kMotorFault)) {
            // return .5f;
            // }
            // // if () {
            // // faults += "MOTOR_FAULT, ";
            // // faultered = true;
            // // }
            // if (theMotor.getFault(FaultID.kCANRX)) {
            // return .5f;
            // }
            // if (theMotor.getFault(FaultID.kCANTX)) {
            // return .5f;
            // }
            // if (theMotor.getFault(FaultID.kSensorFault)) {
            // return .5f;
            // }
            // if (theMotor.getFault(FaultID.kStall)) {
            // return .5f;
            // }
            // if (theMotor.getFault(FaultID.kDRVFault)) {
            // return .5f;
            // }
        }
        return 1.5f;
    }

    static float diagnoseFalcon(TalonFX theMotor) {
        if (!theMotor.getDutyCycle().hasUpdated()) {
            return .5f;
        }
        theMotor.getFaultField().refresh();
        int status = theMotor.getFaultField().getValue();
        if (status != 0) {
            return .5f;
        }
        return 1.5f;
    }
}
