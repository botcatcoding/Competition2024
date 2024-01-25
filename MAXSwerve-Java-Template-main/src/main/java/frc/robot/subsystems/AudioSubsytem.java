package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AudioSubsytem {


    DoublePublisher audioCue;
    public AudioSubsytem(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable audioSubsytem = inst.getTable("audioSubsytem");
        audioCue = audioSubsytem.getDoubleTopic("cue").publish();
    }
    public void sendCue(String cue){
        if(cue.equals("test")){
            audioCue.set(1);

        }
        else if(cue.equals("cancel")){
            audioCue.set(0);
        }

    
    }
}
