package frc.robot.commands.autos;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.mech.ShootToThrill;
import frc.robot.commands.mech.ShutUpAndDanceCmdGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class AutoManager extends Command {
    LinkedList<Command> commands = new LinkedList<Command>();
    int currentCommandIndex = 0;
    Slurp sl;
    Shooter sho;
    Elbow e;
    Shoulder sh;
    Kinamatics k;
    DigitalInput sd;
    DriveSubsystem ds;
    RobotContainer rc;
    Lighting lighting;

    public AutoManager(RobotContainer rc, Lighting lighting, DriveSubsystem ds, Slurp sl, Shooter sho, Elbow e,
            Shoulder sh,
            Kinamatics k,
            DigitalInput sd) {
        this.sl = sl;
        this.sho = sho;
        this.e = e;
        this.sh = sh;
        this.k = k;
        this.sd = sd;
        this.ds = ds;
        this.rc = rc;
        this.lighting = lighting;

    }

    @Override
    public void initialize() {
        currentCommandIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).schedule();
        }
    }

    @Override
    public void execute() {
        if (commands.isEmpty()) {
            return;
        }
        Command currentCommand = commands.get(currentCommandIndex);
        System.out.println(currentCommandIndex + ":" + currentCommand.toString() + ":" + currentCommand.isFinished());
        if (currentCommand.isFinished()) {
            currentCommandIndex++;
            if (currentCommandIndex < commands.size()) {
                commands.get(currentCommandIndex).schedule();
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted
                && !commands.isEmpty()
                && currentCommandIndex > -1
                && currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommandIndex == commands.size();
    }

    String selectedAuto = "INVALID";

    public void setAuto(String auto) {// "Nothing", "One note, run", "Close", "Stage Left Far", "Stage Right Far"
        if (auto.equals("Nothing")) {
            initNothing();
            selectedAuto = auto;
        } else if (auto.equals("One note, run")) {
            initOneNote();
            selectedAuto = auto;
        } else if (auto.equals("Close")) {
            initFourNoteClose();
            selectedAuto = auto;
        } else if (auto.equals("Stage Left Far")) {
            initThreeNoteFarSL();
            selectedAuto = auto;
        } else if (auto.equals("Stage Right Far")) {
            initThreeNoteFarSL();
            selectedAuto = auto;
        }
        selectedAuto = "INVALID";
    }

    public String getAuto() {
        return selectedAuto;
    }

    public void initOneNote() {
        ShootToThrill big1 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        WaitCommand waitup = new WaitCommand(8.14159);
        Command chero1 = rc.constructChoreoCommand("1 note.1");

        commands.add(big1);
        commands.add(waitup);
        commands.add(chero1);

    }

    public void initNothing() {

        ShootToThrill big1 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        commands.add(big1);
    }

    public void initFourNoteClose() {
        ShootToThrill big1 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        Command chero1 = rc.constructChoreoCommand("4 note close.1");
        Command chero2 = rc.constructChoreoCommand("4 note close.2");
        Command chero3 = rc.constructChoreoCommand("4 note close.3");
        Command chero4 = rc.constructChoreoCommand("4 note close.4");
        ShutUpAndDanceCmdGroup suadcg1 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);
        ShutUpAndDanceCmdGroup suadcg2 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);
        ShutUpAndDanceCmdGroup suadcg3 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);
        // LookingForLove lfl = new LookingForLove(ds, e, sh, sd, sl,
        // sho);
        // LookingForLove lfl2 = new LookingForLove(ds, e, sh, sd, sl,
        // sho);
        // LookingForLove lfl3 = new LookingForLove(ds, e, sh, sd, sl,
        // sho);
        ShootToThrill big2 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big3 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big4 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, false);
        commands.add(big1.alongWith(chero1));
        // commands.add();
        commands.add(suadcg1);
        commands.add(big2.alongWith(chero2));
        // commands.add();
        commands.add(suadcg2);
        commands.add(big3.alongWith(chero3));
        commands.add(suadcg3);
        commands.add(big4);
        commands.add(chero4);
    }

    public void initThreeNoteFarSR() {
        ShootToThrill big1 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big2 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big3 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        Command chero1 = rc.constructChoreoCommand("3 note far SR.1");
        Command chero2 = rc.constructChoreoCommand("3 note far SR.2");
        Command chero3 = rc.constructChoreoCommand("3 note far SR.3");
        Command chero4 = rc.constructChoreoCommand("3 note far SR.4");
        ShutUpAndDanceCmdGroup suadcg1 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);
        ShutUpAndDanceCmdGroup suadcg2 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);

        commands.add(big1);
        commands.add(chero1);
        commands.add(suadcg1);
        commands.add(chero2);
        commands.add(big2);
        commands.add(chero3);
        commands.add(suadcg2);
        commands.add(chero4);
        commands.add(big3);

    }

    public void initThreeNoteFarSL() {
        ShootToThrill big1 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big2 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        ShootToThrill big3 = new ShootToThrill(ds, lighting, sl, sho, e, sh, k, sd, true, true, true);
        Command chero1 = rc.constructChoreoCommand("3 note far SL.1");
        Command chero2 = rc.constructChoreoCommand("3 note far SL.2");
        Command chero3 = rc.constructChoreoCommand("3 note far SL.3");
        Command chero4 = rc.constructChoreoCommand("3 note far SL.4");
        ShutUpAndDanceCmdGroup suadcg1 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);
        ShutUpAndDanceCmdGroup suadcg2 = new ShutUpAndDanceCmdGroup(lighting, ds, e, sh, sd, sl, sho);

        commands.add(big1);
        commands.add(chero1);
        commands.add(suadcg1);
        commands.add(chero2);
        commands.add(big2);
        commands.add(chero3);
        commands.add(suadcg2);
        commands.add(chero4);
        commands.add(big3);
    }
}
