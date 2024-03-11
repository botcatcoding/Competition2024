// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAndSpin;
import frc.robot.commands.AimAtSpeakerCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveToSlurpPositionWrapper;
import frc.robot.commands.SetElbowByRotationSafe;
import frc.robot.commands.SetShoulderByRotation;
import frc.robot.commands.SlurpCommand;
import frc.robot.commands.SlurpFromSourceWrapper;
import frc.robot.commands.SlurpItCommandGroup;
import frc.robot.commands.TheBigGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommandGroup;
import frc.robot.subsystems.AudioSubsytem;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Slurp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Diagnostics diagnosics = new Diagnostics(m_robotDrive);
    private final Shooter shooter = new Shooter();
    private final Slurp slurp = new Slurp();
    private final Elbow elbow = new Elbow();
    private final Shoulder shoulder = new Shoulder();
    private Kinamatics kinamatics;

    final DigitalInput slurpDetect = new DigitalInput(2);

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_mechDriver = new Joystick(OIConstants.kMechControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        AudioSubsytem.init();
        AudioSubsytem.playToneTime(440, 5);
        kinamatics = new Kinamatics(m_robotDrive, false);

        // Configure the button bindings
        configureButtonBindings();
        // RunDiagnosticsCommand rdc = new RunDiagnosticsCommand(diagnosics);
        // diagnosics.setDefaultCommand(rdc);
        // rdc.ignoringDisable(true);
        slurp.setDefaultCommand(new SlurpCommand(0.0, false, false, slurp, slurpDetect));
        shooter.setDefaultCommand(new ShootCommand(shooter, 0, false, false));
        elbow.setDefaultCommand(
                new SetElbowByRotationSafe(-.25, false, .01, Constants.MechConstants.shoulderSafeZone, false, shoulder,
                        elbow));
        shoulder.setDefaultCommand(
                new SetShoulderByRotation(Constants.MechConstants.shoulderDefault, false, false, shoulder));

        m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive,
                m_driverController));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // driver
        new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));
        new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_robotDrive.setPointAtYaw(true)))
                .whileFalse(new RunCommand(() -> m_robotDrive.setPointAtYaw(false)));
        // buttons
        new JoystickButton(m_mechDriver, 6)
                .whileTrue(new TheBigGroup(slurp, shooter, elbow, shoulder, kinamatics, slurpDetect));
        new JoystickButton(m_mechDriver, 5).whileTrue(new SlurpCommand(-6000, true, false, slurp, slurpDetect));
        new JoystickButton(m_mechDriver, 3).whileTrue(new SlurpCommand(6000, true, false, slurp, slurpDetect));
        new JoystickButton(m_mechDriver, 4).whileTrue(new SlurpItCommandGroup(slurp, slurpDetect, shooter));
        new JoystickButton(m_mechDriver, 2)
                .whileTrue(new AimAndSpin(shoulder, elbow, kinamatics, shooter));

        // POV
        // new POVButton(m_mechDriver, 0).whileTrue(new
        // MoveToSlurpPositionWrapper(shoulder, elbow, m_robotDrive,
        // m_mechDriver, slurpDetect, slurp));
        // new POVButton(m_mechDriver, 90).whileTrue(new
        // MoveToSlurpPositionWrapper(shoulder, elbow, m_robotDrive,
        // m_mechDriver, slurpDetect, slurp));
        // new POVButton(m_mechDriver, 180).whileTrue(new
        // MoveToSlurpPositionWrapper(shoulder, elbow, m_robotDrive,
        // m_mechDriver, slurpDetect, slurp));
        // new POVButton(m_mechDriver, 270).whileTrue(new
        // MoveToSlurpPositionWrapper(shoulder, elbow, m_robotDrive,
        // m_mechDriver, slurpDetect, slurp));
        new JoystickButton(m_mechDriver, 1)
                .whileTrue(new SlurpFromSourceWrapper(shoulder, elbow, m_robotDrive, slurp, slurpDetect));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("auto1");
    }
}
