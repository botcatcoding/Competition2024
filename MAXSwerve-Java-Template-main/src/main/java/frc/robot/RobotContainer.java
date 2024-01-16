// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.Joystick.;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.RunDiagnosticsCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final Diagnostics diagnosics = new Diagnostics(m_robotDrive);
  private final Shooter shooter = new Shooter();
  private final Feed feed = new Feed();
  private final TankDrive drive = new TankDrive();

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
    drive.setDefaultCommand(new RunCommand(
              () -> drive.drive(
                  -MathUtil.applyDeadband(-m_driverController.getY()-m_driverController.getZ()/4, OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(-m_driverController.getY()+m_driverController.getZ()/4, OIConstants.kDriveDeadband)),
              drive));
    // rdc.ignoringDisable(true);
    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
    //             true, true),
    //         m_robotDrive));
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
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    // new JoystickButton(m_driverController, 1)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.zeroHeading(),
    //         m_robotDrive));
    new JoystickButton(m_driverController, 8).whileTrue(new ShootCommand(1,1, shooter));
    new JoystickButton(m_driverController, 7).whileTrue(new FeedCommand(.5, feed));   
    
    new JoystickButton(m_driverController, 10).whileTrue(new ShootCommand(-.2,-.2, shooter));
    new JoystickButton(m_driverController, 9).whileTrue(new FeedCommand(-.2, feed));   
    
  }

  // PPSwerveControllerCommand constructPPCommand(String pathName, PIDController thetaController)
  // {
  //   PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(1, 1));


  //   // PathPlannerState state = (PathPlannerState) path.sample(autoTimer.get());
  //   // state.holonomicRotation
  //   PPSwerveControllerCommand ppCommand = new PPSwerveControllerCommand(
  //       path,
  //       m_robotDrive::getPose,
  //       DriveConstants.kDriveKinematics,
  //       new PIDController(AutoConstants.kPXController, 0, 0),
  //       new PIDController(AutoConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       false,
  //       m_robotDrive
  //       );
  //   return ppCommand;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {

//     // An example trajectory to follow. All units in meters.
    
//    // PathPlannerTrajectory path = PathPlanner.loadPath("main", new PathConstraints(1, 1));

    

//     // path.getState(0).
//     var thetaController = new PIDController(
//         AutoConstants.kPThetaController, 0, 0);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
//     PPSwerveControllerCommand p1Command = constructPPCommand("P1", thetaController);
//     PPSwerveControllerCommand p2Command = constructPPCommand("P2", thetaController);
//     PPSwerveControllerCommand p3Command = constructPPCommand("P3", thetaController);
//     PPSwerveControllerCommand p4Command = constructPPCommand("P4", thetaController);



//     // Reset odometry to the starting pose of the trajectory.
//     m_robotDrive.resetOdometry(PathPlanner.loadPath("P1", new PathConstraints(1, 1)).getInitialPose());

//     // Run path following command, then stop at the end.
//     return p1Command.andThen(p2Command.andThen(p3Command.andThen(p4Command.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))))).repeatedly();
//   }
}
