package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.OutputStreamWriter;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {

    public Lighting() {
        super();
    }

    static SerialPort myPort;
    static BufferedWriter out;
    static Alliance knownAlliance = null;
    static long loops = 0;
    static long lastTryConnect = 0;
    static boolean disabled = false;
    static boolean canError = false;
    static String knownMode = "";
    static String currentNotification = "";
    static String knownNotification = "";
    static boolean wasTelop = false;

    public static Command constructNotificationCommand(Lighting lighting, String notification) {
        return new NotificationCommand(lighting, notification);
    }

    @Override
    public void periodic() {
        try {
            periodic(DriverStation.isDSAttached(), DriverStation.isEnabled(),
                    DriverStation.isTeleop(),
                    DriverStation.isAutonomous(), DriverStation.getAlliance().get());
        } catch (Exception e) {
            // e.printStackTrace();
        }
    }

    static class NotificationCommand extends Command {
        String noti;

        public NotificationCommand(Lighting lighting, String notification) {
            addRequirements(lighting);
            noti = notification;
        }

        @Override
        public void initialize() {
            currentNotification = noti;
        }

        @Override
        public void execute() {

        }

        @Override
        public void end(boolean interrupted) {
        }

        @Override
        public boolean isFinished() {
            return true;

        }
    }

    static void reset() {
        if (System.currentTimeMillis() - lastTryConnect > 10000
                || (System.currentTimeMillis() - lastTryConnect > 100 && disabled))// only try to reconnect every 10
                                                                                   // seconds, or if disabled, every 100
                                                                                   // millis
        {
            lastTryConnect = System.currentTimeMillis();
            if (myPort != null) {
                if (myPort.isOpen()) {
                    myPort.closePort();
                }
            }
            init();
        }
    }

    public static void init() {
        SerialPort[] ports = SerialPort.getCommPorts();
        myPort = null;
        // myPort = ports[0];
        for (int i = 0; i < ports.length; i++) {
            if (ports[i].getSystemPortName().contains("ACM")) {
                myPort = ports[i];
            }
        }
        if (myPort != null) {
            myPort.setBaudRate(115200);
            out = new BufferedWriter(new OutputStreamWriter(myPort.getOutputStream()));
            myPort.openPort();
        }
    }

    static void write(String str) {
        try {
            // System.out.println("->"+str);
            out.write(str + "\n");
            out.flush();
        } catch (Exception e) {
            e.printStackTrace();
            // System.err.println("lighting error");
            reset();
        }
    }

    static long lastSend = 0;

    public static void periodic(boolean dsAttached, boolean isEnabled, boolean telop, boolean auto,
            Alliance alliance) {
        if (telop && isEnabled) {
            wasTelop = true;
        }
        // System.out.println(timeLeft);
        disabled = !isEnabled;
        String mode = "E";
        if (canError) {
            mode = "C";
        } else {
            if (isEnabled) {
                if (telop) {
                    mode = "T";
                } else if (auto) {
                    mode = "A";
                }
            } else {
                if (wasTelop) {
                    mode = "E";
                } else {
                    mode = "D";
                }
            }
        }
        if (!dsAttached) {
            alliance = null;
        }
        if (!knownNotification.equals(currentNotification)) {
            write("N" + currentNotification);
            knownNotification = currentNotification;
        }
        if (loops % 100 == 0) {
            knownAlliance = null;
            knownMode = "";
            knownNotification = "";
        }
        sendAlliance(alliance);
        sendMode(mode);
        loops++;
    }

    static void sendMode(String mode) {
        if (!knownMode.equals(mode)) {
            write("M" + mode);
        }
        knownMode = mode;
    }

    static void sendAlliance(Alliance alliance) {
        if (knownAlliance != alliance) {
            if (alliance == Alliance.Red) {
                write("AR");
            } else if (alliance == Alliance.Blue) {
                write("AB");
            } else {
                write("A0");
            }
            knownAlliance = alliance;
        }
    }
}