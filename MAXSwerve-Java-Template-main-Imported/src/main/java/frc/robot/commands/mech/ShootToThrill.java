package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.YawToSpeaker;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class ShootToThrill extends DebugedSequentialCommandGroup {

    public ShootToThrill(DriveSubsystem ds, Lighting lighting, Slurp sl, Shooter sho, Elbow e, Shoulder sh,
            Kinamatics k, DigitalInput sd,
            boolean ss, boolean aimDrive, boolean whileChoreo) {

        Command spinNotification = Lighting.constructNotificationCommand(lighting, "R");
        Command shootNotification = Lighting.constructNotificationCommand(lighting, "S");
        if (aimDrive) {
            AimAtSpeakerCommand aasc = new AimAtSpeakerCommand(ds, sh, e, k, true, 5);
            ShootCommandWithLength spinUp = new ShootCommandWithLength(sho, k, true);
            YawToSpeaker yts = new YawToSpeaker(ds, whileChoreo);

            ShootCommandGroup scg = new ShootCommandGroup(sho, sl, sd, k, ss);
            AimAtSpeakerCommand aascns = new AimAtSpeakerCommand(ds, sh, e, k, false, 5);
            YawToSpeaker yts2 = new YawToSpeaker(ds, whileChoreo);

            Command aimAndShoot = scg.deadlineWith(aascns, yts, shootNotification);
            Command aimAndSpin = aasc.deadlineWith(spinUp, yts2, spinNotification);

            addCommands(aimAndSpin, aimAndShoot);
        } else {
            AimAtSpeakerCommand aasc = new AimAtSpeakerCommand(ds, sh, e, k, true, 10);
            ShootCommandWithLength spinUp = new ShootCommandWithLength(sho, k, true);

            ShootCommandGroup scg = new ShootCommandGroup(sho, sl, sd, k, ss);
            AimAtSpeakerCommand aascns = new AimAtSpeakerCommand(ds, sh, e, k, false, 10);

            Command aimAndShoot = scg.deadlineWith(aascns, shootNotification);
            Command aimAndSpin = aasc.deadlineWith(spinUp, spinNotification);

            addCommands(aimAndSpin, aimAndShoot);
        }
    }

}
