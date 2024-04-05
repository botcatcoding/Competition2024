// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autos.AutoManager;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.mech.AimAndSpin;
import frc.robot.commands.mech.CenterNoteConditional;
import frc.robot.commands.mech.ClimberCommands;
import frc.robot.commands.mech.ImHoldingOutForAHero;
import frc.robot.commands.mech.MoveToSlurpPositionWrapper;
import frc.robot.commands.mech.AmpScoreCommandGroupWrapper;
import frc.robot.commands.mech.SetElbowByRotationSafe;
import frc.robot.commands.mech.SetShoulderByRotation;
import frc.robot.commands.mech.ShootCommand;
import frc.robot.commands.mech.ShootToThrill;
import frc.robot.commands.mech.ShutUpAndDanceCmdGroup;
import frc.robot.commands.mech.SlurpCommand;
import frc.robot.commands.mech.ThinkAboutItAndSlurp;
import frc.robot.subsystems.AudioSubsytem;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Kinamatics;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Slurp;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

        // The robot's subsystems\
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final Shooter shooter = new Shooter();
        private final Slurp slurp = new Slurp();
        private final Elbow elbow = new Elbow();
        private final Shoulder shoulder = new Shoulder();
        private final Lighting lighting = new Lighting();
        private Kinamatics kinamatics = new Kinamatics(m_robotDrive);;
        public final DigitalInput slurpDetect = new DigitalInput(2);
        AutoManager autoCommand = new AutoManager(this, lighting, m_robotDrive, slurp, shooter, elbow, shoulder,
                        kinamatics,
                        slurpDetect);
        private final Diagnostics diagnosics = new Diagnostics(m_robotDrive, elbow, shooter, shoulder, slurp,
                        autoCommand);

        private final SendableChooser<Command> autoChooser;
        // The driver's controller
        Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
        Joystick m_mechDriver = new Joystick(OIConstants.kMechControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        ChoreoTrajectory traj;

        public boolean shouldRun() {
                // System.out.println(CenterNoteConditional.intaking);
                return CenterNoteConditional.intaking;
        }

        public Command constructChoreoCommand(String traString) {
                traj = Choreo.getTrajectory(traString);
                return Choreo.choreoSwerveCommand(
                                traj,
                                m_robotDrive::getPose,
                                new PIDController(Constants.AutoConstants.kPTraslation, 0.0, 0.0),
                                new PIDController(Constants.AutoConstants.kPTraslation, 0.0, 0.0),
                                new PIDController(Constants.AutoConstants.kPTheta, 0.0, 0.0),
                                (ChassisSpeeds speeds) -> m_robotDrive
                                                .driveRobotRelative(speeds),
                                () -> {
                                        return false;
                                },
                                m_robotDrive);
        }

        public RobotContainer() {

                // Configure the button bindings
                configureButtonBindings();
                // RunDiagnosticsCommand rdc = new RunDiagnosticsCommand(diagnosics);
                // diagnosics.setDefaultCommand(rdc);
                // rdc.ignoringDisable(true);
                lighting.setDefaultCommand(Lighting.constructNotificationCommand(lighting, "0"));
                slurp.setDefaultCommand(new SlurpCommand(0.0, false, false, slurp,
                                slurpDetect));
                shooter.setDefaultCommand(new ShootCommand(shooter, 0, false, false));
                elbow.setDefaultCommand(
                                new SetElbowByRotationSafe(.2, false, .01,
                                                Constants.MechConstants.shoulderSafeZone, false, shoulder,
                                                elbow));
                shoulder.setDefaultCommand(
                                new SetShoulderByRotation(Constants.MechConstants.shoulderDefault, false,
                                                false, shoulder,
                                                Constants.MechConstants.shoulderDeadband));

                m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive,
                                m_driverController));

                Trigger slurpTrigger = new Trigger(slurpDetect::get);
                slurpTrigger.onTrue(new ThinkAboutItAndSlurp(lighting, slurp, slurpDetect, shooter));

                autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
                SmartDashboard.putData("Auto Mode", autoChooser);
                AudioSubsytem.init();
                // AudioSubsytem.playToneTime(440, 5);
                diagnosics.publish();
                diagnosics.init();

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
                // new JoystickButton(m_driverController, 2).whileTrue(new RunCommand(() ->
                // m_robotDrive.zeroHeading()));
                new JoystickButton(m_driverController, 1)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setPointAtYaw(true)))
                                .whileFalse(new RunCommand(() -> m_robotDrive.setPointAtYaw(false)));
                new JoystickButton(m_driverController, 2)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setPointAtBucket(true)))
                                .whileFalse(new RunCommand(() -> m_robotDrive.setPointAtBucket(false)));
                // buttons
                new JoystickButton(m_mechDriver, 6)
                                .whileTrue(new ShootToThrill(m_robotDrive, lighting, slurp, shooter, elbow, shoulder,
                                                kinamatics,
                                                slurpDetect, false, false, false));
                new JoystickButton(m_mechDriver, 5)
                                .whileTrue(new AmpScoreCommandGroupWrapper(m_robotDrive, slurp, elbow, shoulder,
                                                slurpDetect));
                new JoystickButton(m_mechDriver, 4).whileTrue(new SlurpCommand(6000, true,
                                false, slurp, slurpDetect));
                new JoystickButton(m_mechDriver, 2)
                                .whileTrue(new AimAndSpin(m_robotDrive, shoulder, elbow, kinamatics, shooter));
                // new JoystickButton(m_mechDriver, 3)
                // .whileTrue(new RunCommand(() -> elbow.setPerc(m_mechDriver.getRawAxis(1)),
                // elbow));
                new JoystickButton(m_driverController, 10)
                                .whileTrue(new ShutUpAndDanceCmdGroup(lighting, m_robotDrive, elbow, shoulder,
                                                slurpDetect, slurp,
                                                shooter));
                // new JoystickButton(m_driverController, 3).whileTrue(new
                // DriveToNote(m_robotDrive));

                // AXIS
                BooleanEvent shouldBucket = m_mechDriver.axisGreaterThan(3, .5,
                                CommandScheduler.getInstance().getActiveButtonLoop());
                shouldBucket.castTo(Trigger::new)
                                .whileTrue(new ImHoldingOutForAHero(shooter, elbow, shoulder, slurp, slurpDetect,
                                                kinamatics));

                BooleanEvent shouldClimbUp = m_mechDriver.axisLessThan(1, -.5,
                                CommandScheduler.getInstance().getActiveButtonLoop());
                BooleanEvent shouldClimbDown = m_mechDriver.axisGreaterThan(1, .5,
                                CommandScheduler.getInstance().getActiveButtonLoop());
                ClimberCommands.ClimbOver co = new ClimberCommands.ClimbOver(shoulder, elbow);
                shouldClimbUp.castTo(Trigger::new)
                                .whileTrue(new ConditionalCommand(
                                                new ClimberCommands.ClimbUpGroup(shoulder, elbow, false),
                                                new WaitCommand(0),
                                                m_mechDriver.button(1,
                                                                CommandScheduler.getInstance().getActiveButtonLoop())))
                                .whileFalse(co);
                shouldClimbDown.castTo(Trigger::new)
                                .whileTrue(new ConditionalCommand(
                                                new ClimberCommands.AintNoMountainHighEnough(shoulder, elbow, false),
                                                new WaitCommand(0),
                                                m_mechDriver.button(1,
                                                                CommandScheduler.getInstance().getActiveButtonLoop())))
                                .whileFalse(co);
                // // POV
                new POVButton(m_mechDriver, 0)
                                .whileTrue(new MoveToSlurpPositionWrapper(lighting, shoulder, elbow, shooter,
                                                m_robotDrive,
                                                m_mechDriver, slurpDetect, slurp));
                new POVButton(m_mechDriver, 90)
                                .whileTrue(new MoveToSlurpPositionWrapper(lighting, shoulder, elbow, shooter,
                                                m_robotDrive,
                                                m_mechDriver, slurpDetect, slurp));
                new POVButton(m_mechDriver, 180)
                                .whileTrue(new MoveToSlurpPositionWrapper(lighting, shoulder, elbow, shooter,
                                                m_robotDrive,
                                                m_mechDriver, slurpDetect, slurp));
                new POVButton(m_mechDriver, 270)
                                .whileTrue(new MoveToSlurpPositionWrapper(lighting, shoulder, elbow, shooter,
                                                m_robotDrive,
                                                m_mechDriver, slurpDetect, slurp));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoCommand;
        }
}
