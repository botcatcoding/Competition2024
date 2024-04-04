package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class TheSmallGroup extends SequentialCommandGroup {

    public TheSmallGroup(Lighting lighting, Slurp sl, Shooter sho, Elbow e, Shoulder sh, DigitalInput sd,
            boolean inverted) {
        MoveToSlurpPositionWrapper.MoveToSlurpPosition mtsp = new MoveToSlurpPositionWrapper.MoveToSlurpPosition(
                inverted, false, sh, e, lighting);
        SlurpCommand slurp = new SlurpCommand(-6000, true, false, sl, sd);
        Command both = slurp.deadlineWith(mtsp);
        CenterNoteConditional thinkAboutCentering = new CenterNoteConditional();
        Command bothboth = both.alongWith(thinkAboutCentering);
        addCommands(bothboth);

    }

}
