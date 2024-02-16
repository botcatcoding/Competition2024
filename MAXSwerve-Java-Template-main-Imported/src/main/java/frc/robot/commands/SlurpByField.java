package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Slurp;

public class SlurpByField extends Command {
    double speed = 0;
    Slurp slurp;
    Arm arm;
    DigitalInput slurpDetect;
    Joystick joystick;
    boolean velocityMode;
    boolean stopIfLimitSwitch;
    DriveSubsystem iWantYaw;

    public SlurpByField(double s, boolean vm, boolean ss, Slurp sl, DigitalInput sd, Joystick js, Arm a,
            DriveSubsystem yaw) {
        addRequirements(sl);
        speed = s;
        slurp = sl;
        joystick = js;
        velocityMode = vm;
        stopIfLimitSwitch = ss;
        arm = a;
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
            arm.setPosition(90, 270);
        } else {
            arm.setPosition(90, 90);
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
