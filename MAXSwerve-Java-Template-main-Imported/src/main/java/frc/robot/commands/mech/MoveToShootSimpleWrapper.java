package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class MoveToShootSimpleWrapper extends Command {

    public class MoveToShootSimple extends ParallelCommandGroup {

        public MoveToShootSimple(boolean inverted, Shoulder s, Elbow e) {

            double shoulderAngle = .3;
            double elbowAngle = .35;
            // indivual commands
            SetShoulderByRotation shoulderMovement = new SetShoulderByRotation(
                    shoulderAngle,
                    false, inverted,
                    s, Constants.MechConstants.shoulderDeadband);

            SetElbowByRotationSafe elbowMovement = new SetElbowByRotationSafe(elbowAngle, true, .05,
                    Constants.MechConstants.shoulderSafeZone, inverted, s, e);

            // commands running at the same time
            addCommands(shoulderMovement, elbowMovement);

        }
    }

    MoveToShootSimple mtss;
    Shoulder shoulder;
    Elbow elbow;
    DriveSubsystem m_robotDrive;
    Slurp slurp;
    Joystick m_mechDriver;
    boolean shouldInvert;

    public MoveToShootSimpleWrapper(Shoulder s, Elbow e, DriveSubsystem ds) {
        shoulder = s;
        elbow = e;
        m_robotDrive = ds;

    }

    @Override
    public void initialize() {
        shouldInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        mtss = new MoveToShootSimple(shouldInvert, shoulder, elbow);
        mtss.schedule();
    }

    @Override
    public void execute() {
        boolean nowInvert = Slurp.slurpFieldOrient(m_mechDriver.getPOV(), m_robotDrive.getHeading());
        if (shouldInvert != nowInvert) {
            shouldInvert = nowInvert;
            mtss.cancel();
            mtss = new MoveToShootSimple(shouldInvert, shoulder, elbow);
            mtss.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mtss.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
