package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Slurp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.DriveToNote;

public class LookingForLove extends Command {
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem driveSubsystem;
    DriveToNote dtn;
    ShutUpAndDanceCmdGroup suadcg;

    public LookingForLove(DriveSubsystem ds, Lighting lighting, Elbow e, Shoulder sh, DigitalInput slurpDetect,
            Slurp sl, Shooter sho) {
        addRequirements(e, sh);
        shoulder = sh;
        elbow = e;
        driveSubsystem = ds;
        suadcg = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, slurpDetect, sl, sho);
    }

    @Override
    public void initialize() {
        dtn = new DriveToNote(driveSubsystem);
        timeSeen = 0;
        scheduled = false;
        scheduledSUAD = false;
        dtnHasFinished = false;
        suadcgHasFinished = false;
    }

    int timeSeen = 0;
    boolean scheduled = false;
    boolean scheduledSUAD = false;
    boolean dtnHasFinished = false;
    boolean suadcgHasFinished = false;

    @Override
    public void execute() {
        double elbowpos = 0;
        double shoulderpos = .25;
        // shoulder
        shoulder.setPosition(shoulderpos);
        // elbow
        double shoulderAbsoluteAngle = Math.abs(shoulder.getShoulderAngle() - Constants.MechConstants.shoulderMiddle);
        if (shoulderAbsoluteAngle < Constants.MechConstants.shoulderSafeZone) {// we can move because we are in the safe
                                                                               // zone
            elbow.setPosition(elbowpos);
            SmartDashboard.putString("safety", "SAFE");
        } else {// we cant move rn
            elbow.dontMove();
            SmartDashboard.putString("safety", "UNSAFE");
        }
        if (LimelightHelpers.getTV(Constants.aOrangelightName)) {
            timeSeen++;
        } else {
            timeSeen = 0;
        }
        // System.out.println(timeSeen);
        if (timeSeen > 10
                && !dtn.isScheduled()) {
            dtn.schedule();
            scheduled = true;
        }
        if (dtn.isFinished() && scheduled) {
            dtn.cancel();
            cancel();
            suadcg.schedule();
            scheduledSUAD = true;
        }
        dtnHasFinished = dtnHasFinished || (dtn.isFinished() && scheduled);
        suadcgHasFinished = suadcgHasFinished || (suadcg.isFinished() && scheduledSUAD);
    }

    boolean hasEnded = false;

    @Override
    public void end(boolean interrupted) {
        dtn.cancel();
        suadcg.cancel();
    }

    @Override
    public boolean isFinished() {
        // System.out.println(dtnHasFinished + ":" + dtn.isFinished() + ":" +
        // suadcgHasFinished + ":" + suadcg.isFinished());
        return dtnHasFinished && suadcg.isFinished();
    }
}
