package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AudioSubsytem {

    static DoublePublisher audioTone;
    static int playingId = -1;

    public static void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable audioSubsytem = inst.getTable("audioSubsytem");
        audioTone = audioSubsytem.getDoubleTopic("tone").publish();
    }

    static public void playTone(int hz) {
        audioTone.set(hz);
    }

    static public void playToneTime(int hz, double seconds) {
        playTone(hz);
        playingId++;
        setupStop(playingId, seconds);
    }

    static void setupStop(int id, double seconds) {
        new Thread(new EndThread(id, seconds)).start();
    }

    static class EndThread implements Runnable {
        int id;
        double seconds;

        public EndThread(int id, double seconds) {
            this.id = id;
            this.seconds = seconds;
        }

        @Override
        public void run() {
            try {
                Thread.sleep((int) ((seconds) * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (playingId == id) {
                playTone(0);
            }
        }

    }
}
