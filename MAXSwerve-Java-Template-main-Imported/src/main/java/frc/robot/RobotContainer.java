// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final Feed feed = new Feed();
  private final Arm arm = new Arm();
 // private final TankDrive drive = new TankDrive();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    
    
    // Configure the button bindings 
    configureButtonBindings();
    // RunDiagnosticsCommand rdc = new RunDiagnosticsCommand(diagnosics);
    // diagnosics.setDefaultCommand(rdc);
    feed.setDefaultCommand(new FeedCommand(0, feed));
    shooter.setDefaultCommand(new ShootCommand(0,0, shooter));
    // rdc.ignoringDisable(true);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
    //    // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
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
    new JoystickButton(m_driverController, 1)
        .whileTrue(new AimAndDriveCommand(m_robotDrive, arm, m_driverController));
    new JoystickButton(m_driverController, 8).whileTrue(new ShootCommand(1,1, shooter));
    new JoystickButton(m_driverController, 7).whileTrue(new FeedCommand(.5, feed));   
    
    new JoystickButton(m_driverController, 10).whileTrue(new ShootCommand(-.2,-.2, shooter));
    new JoystickButton(m_driverController, 9).whileTrue(new FeedCommand(-.2, feed));   
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("auto");
  }
}
