// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Pigeon2 theGyro = new Pigeon2(DriveConstants.gyroCanId);
    private Field2d m_field = new Field2d();
    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    public boolean pointAtYaw = false;
    public double theYaw = 0;
    public ProfiledPIDController profiledPIDController = new ProfiledPIDController(AutoConstants.kPTheta, 0, 0,
            new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed * 2));
    // Odometry class for tracking robot pose

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(theGyro.getYaw().getValue()),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            }, new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // chassis speeds
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    // chassis speeds field
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds
                .fromRobotRelativeSpeeds(DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                        m_frontRight.getState(),
                        m_rearLeft.getState(),
                        m_rearRight.getState()), poseEstimator.getEstimatedPosition().getRotation());
    }

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        zeroHeading();
        SmartDashboard.putData("Field", m_field);

        profiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        profiledPIDController.reset(getPose().getRotation().getRadians());
        // this is where path planner goes

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                Constants.AutoConstants.kHolonomicPathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    double areaToStd(double area, double x) {
        double distFromZero = Math.abs(x);
        double areaSub = distFromZero / 10000;
        area = area - areaSub;
        // SmartDashboard.putNumber("area", area);
        double MIN_VAL = 0.02;
        double MIN_AREA = 0.0015;
        if (area < MIN_AREA) {
            area = MIN_AREA + 0.0001;
        }
        double equ = (1 / ((area - MIN_AREA) / 2)) / 2000 - .15;
        if (equ < MIN_VAL) {
            equ = MIN_VAL;
        }
        // System.out.println(area + ":" + equ);
        return equ;
    }

    public boolean isPostioned(double deadband) {

        return Math.abs(profiledPIDController.getPositionError()) < Rotation2d.fromDegrees(deadband).getRadians();
    }

    public void setPointAtYaw(boolean pay) {
        pointAtYaw = pay;
        if (pointAtYaw) {
            theYaw = Kinamatics.yaw;
        }
    }

    public void setPointAtBucket(boolean pay) {
        pointAtYaw = pay;
        if (pointAtYaw) {
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
                theYaw = .55;
            } else
                theYaw = 2.6;
        }
    }

    public void resetProfiled() {
        profiledPIDController.reset(poseEstimator.getEstimatedPosition().getRotation().getRadians());
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // theGyro.setYaw(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
            poseEstimator.resetPosition(
                    Rotation2d.fromDegrees(theGyro.getYaw().getValue()),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                    },
                    getPose());
        }
        theGyro.getYaw().refresh();
        SmartDashboard.putNumber("gyroYaw", theGyro.getYaw().getValue());
        // Update the odometry in the periodic block
        poseEstimator.update(
                Rotation2d.fromDegrees(theGyro.getYaw().getValue()),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        if (LimelightHelpers.getTV(Constants.aLimelightName)) {
            double snapshotTime = Timer.getFPGATimestamp()
                    - LimelightHelpers.getLatency_Capture(Constants.aLimelightName) / 1000.0
                    - LimelightHelpers.getLatency_Pipeline(Constants.aLimelightName) / 1000.0;
            Pose2d limeLightBotPose = LimelightHelpers.getBotPose2d(Constants.aLimelightName);
            Pose2d limeLightCorrected = new Pose2d(limeLightBotPose.getX() + Constants.limeLightxOff,
                    limeLightBotPose.getY() + Constants.limeLightyOff, limeLightBotPose.getRotation());
            double std = areaToStd(LimelightHelpers.getTA(Constants.aLimelightName) / 120,
                    LimelightHelpers.getTX(Constants.aLimelightName));
            poseEstimator.addVisionMeasurement(limeLightCorrected, snapshotTime,
                    VecBuilder.fill(std, std, Units.degreesToRadians(50)));
        }
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
        // System.out.println(getPose().getX() + "\t" + getPose().getY());
        // System.out.println(LimelightHelpers.getTV(Constants.aLimelightName)+"\t"+getPose().getX()+"\t"+getPose().getY());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        System.out.println("RESET POSE");
        System.out.println(pose);
        // poseEstimator.resetPosition(
        // Rotation2d.fromDegrees(theGyro.getYaw().getValue()),
        // new SwerveModulePosition[] {
        // m_frontLeft.getPosition(),
        // m_frontRight.getPosition(),
        // m_rearLeft.getPosition(),
        // m_rearRight.getPosition()
        // },
        // pose);
    }

    public void driveFieldRelative(ChassisSpeeds fieldChassisSpeedsIn) {
        ChassisSpeeds cs = ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeedsIn,
                getPose().getRotation());
        driveRobotRelative(cs);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeedsIn) {

        SmartDashboard.putNumber("targetYaw", theYaw);
        SmartDashboard.putNumber("poseYaw", getPose().getRotation().getRadians());
        double yawCommandToTarget = profiledPIDController.calculate(getPose().getRotation().getRadians(),
                theYaw);
        if (pointAtYaw) {
            chassisSpeedsIn.omegaRadiansPerSecond = yawCommandToTarget;
        }

        SwerveModuleState[] sms = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeedsIn);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                sms, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(sms[0]);
        m_frontRight.setDesiredState(sms[1]);
        m_rearLeft.setDesiredState(sms[2]);
        m_rearRight.setDesiredState(sms[3]);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        driveFieldRelative(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        theGyro.setYaw(0);
        // poseEstimator
    }

    public double getHeading() {
        return getPose().getRotation().getDegrees();
    }

    public String diagnostic() {
        MAXSwerveModule[] modules = new MAXSwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };
        String faults = "";
        boolean faultered = false;
        for (int i = 0; i < modules.length; i++) {
            String moduleFault = modules[i].diagnostic();
            if (moduleFault != null) {
                faults += moduleFault + ", ";
                faultered = true;
            }
        }
        if (faultered) {
            return faults;
        }
        return null;
    }

}
