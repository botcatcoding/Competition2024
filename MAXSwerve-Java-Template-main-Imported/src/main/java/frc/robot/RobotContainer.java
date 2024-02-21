// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveToSlurpPosition;
import frc.robot.commands.MoveToSlurpPositionWrapper;
import frc.robot.commands.SetElbowByRotationSafe;
import frc.robot.commands.SetShoulderByRotation;
// import frc.robot.commands.SetElbowAndShoulderByPositionCommand;
import frc.robot.commands.SlurpCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Slurp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    final DigitalInput slurpDetect = new DigitalInput(2);

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_mechDriver = new Joystick(OIConstants.kMechControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // NamedCommands.registerCommand("slurp", Slurp.slurp(slurp));
        NamedCommands.registerCommand("shoot", Shooter.shoot(shooter));
        // NamedCommands.registerCommand("gotoSlurp", Arm.gotoSlurpPosition(arm));

        // Configure the button bindings
        configureButtonBindings();
        // RunDiagnosticsCommand rdc = new RunDiagnosticsCommand(diagnosics);
        // diagnosics.setDefaultCommand(rdc);
        slurp.setDefaultCommand(new SlurpCommand(0, false, false, slurp, slurpDetect));
        shooter.setDefaultCommand(new ShootCommand(0, shooter));
        elbow.setDefaultCommand(
                new SetElbowByRotationSafe(.25, false, .01, Constants.MechConstants.shoulderSafeZone, false, shoulder,
                        elbow));
        shoulder.setDefaultCommand(
                new SetShoulderByRotation(Constants.MechConstants.shoulderDefault, false, false, shoulder));

        // arm.setDefaultCommand(new SetElbowAndShoulderByPositionCommand(90, 0, arm));
        // rdc.ignoringDisable(true);
        // Configure default commands
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
        // new JoystickButton(m_driverController, 1)
        // .whileTrue(new AimAndDriveCommand(m_robotDrive, arm, m_driverController));
        new JoystickButton(m_mechDriver, 6).whileTrue(new ShootCommand(1,
                shooter));
        // new JoystickButton(m_driverController, 10).whileTrue(new
        // ShootCommandByAxis(m_driverController,
        // shooter));
        new JoystickButton(m_mechDriver, 5).whileTrue(new SlurpCommand(-1400, true, false, slurp, slurpDetect));
        new JoystickButton(m_mechDriver, 3).whileTrue(new SlurpCommand(1200, true, false, slurp, slurpDetect));

        new JoystickButton(m_mechDriver, 4)
                .whileTrue(new MoveToSlurpPositionWrapper(shoulder, elbow, m_robotDrive,
                        m_mechDriver));
        // new JoystickButton(m_mechDriver, 2)
        // .whileTrue(new MoveToSlurpPosition(shoulder, elbow, m_robotDrive,
        // m_mechDriver));

        // new JoystickButton(m_mechDriver, 1).whileTrue(new
        // SetElbowAndShoulderByPositionCommand(80, 150, arm));
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
