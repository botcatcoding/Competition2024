package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DigitalPWMSim;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class ImHoldingOutForAHero extends SequentialCommandGroup {
        Shooter shooter;
        Elbow elbow;
        Shoulder shoulder;
        Slurp slurp;
        DigitalInput slurpDetect;
        Kinamatics k;

        public ImHoldingOutForAHero(Shooter sho, Elbow e, Shoulder sh, Slurp sl, DigitalInput sd, Kinamatics k) {
                SetShoulderByRotation moveShoulderToPos = new SetShoulderByRotation(.3, true, false, sh, .1);
                SetElbowByRotationSafe moveElbowToPos = new SetElbowByRotationSafe(.32, true, .1,
                                Constants.MechConstants.shoulderSafeZone, false, sh, e);
                SetShoulderByRotation holdShoulderAtPos = new SetShoulderByRotation(.3, true, false, sh, .1);
                SetElbowByRotationSafe holdElbowAtPos = new SetElbowByRotationSafe(.32, true, .1,
                                Constants.MechConstants.shoulderSafeZone, false, sh, e);
                ShootCommandWithLength spinUp = new ShootCommandWithLength(sho, k, true);
                ShootCommandWithLength keepUp = new ShootCommandWithLength(sho, k, true);
                WaitCommand sleep = new WaitCommand(1);
                SlurpCommand dontFeedIt = new SlurpCommand(0, true, false, sl, sd);
                SlurpCommand feedIt = new SlurpCommand(-6000, true, false, sl, sd);

                ParallelCommandGroup shootIt = new ParallelCommandGroup(feedIt, keepUp, holdShoulderAtPos,
                                holdElbowAtPos);
                ParallelCommandGroup spinUpButDontFeedIt = new ParallelCommandGroup(spinUp, moveShoulderToPos,
                                moveElbowToPos);
                addCommands(spinUpButDontFeedIt, sleep.deadlineWith(shootIt));
        }

}
