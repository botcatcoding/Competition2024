package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class AmpScoreCommandGroupWrapper extends Command {

    public class AmpScoreCommandGroup extends SequentialCommandGroup {
        public AmpScoreCommandGroup(boolean inverted, Slurp sl, Elbow e, Shoulder sh, DigitalInput sd) {
            SetShoulderByRotation moveShoulder = new SetShoulderByRotation(.25, true, inverted, sh,
                    Constants.MechConstants.shoulderDeadband * 2);
            SetElbowByRotationSafe moveElbow = new SetElbowByRotationSafe(-.1, true,
                    Constants.MechConstants.elbowDeadband * 2,
                    Constants.MechConstants.shoulderSafeZone, inverted, sh, e);
            SetElbowByRotationSafe moveElbow2 = new SetElbowByRotationSafe(-.15, true,
                    Constants.MechConstants.elbowDeadband * 2,
                    Constants.MechConstants.shoulderSafeZone, inverted, sh, e);
            SlurpCommand fireIt = new SlurpCommand(4000, true, false, sl, sd);
            SetShoulderByRotation moveThree = new SetShoulderByRotation(.39, true, inverted, sh,
                    Constants.MechConstants.shoulderDeadband * 2);
            WaitCommand wait = new WaitCommand(.15);
            Command set = moveShoulder.alongWith(moveElbow);
            Command fire = moveThree.alongWith(moveElbow2).alongWith(wait.andThen(fireIt));

            addCommands(set, fire);
        }

    }

    public AmpScoreCommandGroupWrapper(DriveSubsystem ds, Slurp sl, Elbow e, Shoulder sh, DigitalInput sd) {
        slurp = sl;
        elbow = e;
        shoulder = sh;
        slurpDetect = sd;
        driveSubsystem = ds;
    }

    DriveSubsystem driveSubsystem;
    AmpScoreCommandGroup ascg;
    Shoulder shoulder;
    Elbow elbow;
    Slurp slurp;
    DigitalInput slurpDetect;
    boolean shouldInvert;

    @Override
    public void initialize() {
        // shouldInvert = Slurp.slurpFieldOrient(90, driveSubsystem.getHeading());
        ascg = new AmpScoreCommandGroup(true, slurp, elbow, shoulder, slurpDetect);
        ascg.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        ascg.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
