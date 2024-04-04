package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;
import frc.robot.Constants;

public class MoveToSlurpPositionWrapper extends Command {
    static public class MoveToSlurpPosition extends SequentialCommandGroup {

        public MoveToSlurpPosition(boolean inverted, boolean shouldEnd, Shoulder s, Elbow e, Lighting lighting) {
            Command slurpLight = Lighting.constructNotificationCommand(lighting, "L");
            // indivual commands
            SetShoulderByRotation shoulderToSafe = new SetShoulderByRotation(
                    Constants.MechConstants.shoulderMiddle + Constants.MechConstants.shoulderSafeZone
                            - Constants.MechConstants.shoulderSaveZoneEdge,
                    false, inverted,
                    s, .1);

            double slurpElbow = .12;
            double slurpShoulder = .5;
            SetElbowByRotationSafe elbowToSlurp = new SetElbowByRotationSafe(slurpElbow, true, .1,
                    Constants.MechConstants.shoulderSafeZone, inverted, s, e);
            SetElbowByRotationSafe elbowToSlurpNoEnd = new SetElbowByRotationSafe(slurpElbow, shouldEnd, .1,
                    Constants.MechConstants.shoulderSafeZone, inverted, s, e);
            SetShoulderByRotation shoulderToSlurp = new SetShoulderByRotation(slurpShoulder, shouldEnd, inverted, s,
                    .1);

            // commands running at the same time
            Command shouldToSafeDeadline = elbowToSlurp.deadlineWith(shoulderToSafe);
            Command shoulderToSlurpAndElbowToSlurp = Commands.parallel(shoulderToSlurp, elbowToSlurpNoEnd);

            // sequenced
            addCommands(slurpLight, shouldToSafeDeadline, shoulderToSlurpAndElbowToSlurp);
        }
    }

    TheSmallGroup tsg;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Slurp slurp;
    Joystick m_mechDriver;
    DigitalInput slurpdetect;
    Lighting lighting;
    boolean shouldInvert;
    Shooter shooter;

    public MoveToSlurpPositionWrapper(Lighting light, Shoulder s, Elbow e, Shooter sho, DriveSubsystem ds,
            Joystick js, DigitalInput sd, Slurp sl) {
        shoulder = s;
        elbow = e;
        m_robotDrive = ds;
        m_mechDriver = js;
        slurpdetect = sd;
        slurp = sl;
        shooter = sho;
        lighting = light;

    }

    @Override
    public void initialize() {
        shouldInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        // SmartDashboard.putBoolean("shouldInvert", shouldInvert);
        tsg = new TheSmallGroup(lighting, slurp, shooter, elbow, shoulder, slurpdetect, shouldInvert);
    }

    @Override
    public void execute() {
        boolean nowInvert = !Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        if (shouldInvert != nowInvert) {
            shouldInvert = nowInvert;
            tsg.cancel();
            tsg = new TheSmallGroup(lighting, slurp, shooter, elbow, shoulder, slurpdetect, shouldInvert);
            tsg.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        tsg.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
