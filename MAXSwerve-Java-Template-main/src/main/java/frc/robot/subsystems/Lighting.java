package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.OutputStreamWriter;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Lighting {
    static SerialPort myPort;
    static BufferedWriter out;
    static Alliance knownAlliance = null;
    static long loops = 0;
    static long lastTryConnect = 0;
    static boolean disabled = false;
    static boolean canError = false;
    static String knownMode = "";
    static boolean wasTelop = false;

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
            System.err.println("lighting error");
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
        if (!canError) {
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
            alliance = Alliance.Invalid;
        }
        if (loops % 25 == 0) {
            knownAlliance = null;
            knownMode = "";
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