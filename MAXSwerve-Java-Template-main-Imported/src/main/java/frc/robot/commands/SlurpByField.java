package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

public class SlurpByField extends Command {
    double speed = 0;
    Slurp slurp;
    Shoulder shoulder;
    Elbow elbow;
    DigitalInput slurpDetect;
    Joystick joystick;
    boolean velocityMode;
    boolean stopIfLimitSwitch;
    DriveSubsystem iWantYaw;

    public SlurpByField(double s, boolean vm, boolean ss, Slurp sl, DigitalInput sd, Joystick js, Shoulder sh, Elbow e,
            DriveSubsystem yaw) {
        addRequirements(sl, e, sh);
        speed = s;
        slurp = sl;
        joystick = js;
        velocityMode = vm;
        stopIfLimitSwitch = ss;
        shoulder = sh;
        elbow = e;
        iWantYaw = yaw;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        slurp.setFeedPercent(.2);
        boolean inverted = Slurp.slurpFieldOrient(joystick.getPOV(), iWantYaw.getHeading());
        if (inverted) {
            // arm.setPosition(90, 270);
        } else {
            // arm.setPosition(90, 90);
        }

    }

    @Override
    public void end(boolean interrupted) {
        slurp.setFeedPercent(0);
    }

    @Override
    public boolean isFinished() {
        return slurpDetect.get() && stopIfLimitSwitch;

    }

}
