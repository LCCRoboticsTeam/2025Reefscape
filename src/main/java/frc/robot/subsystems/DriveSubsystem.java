// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
//import edu.wpi.first.wpilibj.ADIS16470_IMU
import com.studica.frc.AHRS;

import frc.robot.Constants.DriveConstants;
//import frc.robot.Robot;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          //Rotation2d.fromDegrees(-1*m_gyro.getYaw()),
          Rotation2d.fromDegrees((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  // Odometry class for tracking robot pose
  //SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //    DriveConstants.kDriveKinematics,
  //    //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
  //    Rotation2d.fromDegrees(m_gyro.getYaw()),
  //    new SwerveModulePosition[] {
  //        m_frontLeft.getPosition(),
  //        m_frontRight.getPosition(),
  //        m_rearLeft.getPosition(),
  //        m_rearRight.getPosition()
  //    });

  private final Field2d m_field = new Field2d();

  // The field from AprilTagFields will be different depending on the game.
  // Note that kDefaultField is "Welded" which is what is expected at Regional and Champs
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final PhotonPoseEstimator photonPoseEstimatorFrontsideCam;
  private final PhotonPoseEstimator photonPoseEstimatorBacksideCam;
  private Matrix<N3, N1> curStdDevs;
  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private final Transform3d robotToFrontsidePhotonCamera;
  private final Transform3d robotToBacksidePhotonCamera;
  private final PhotonCamera frontsidePhotonCamera;
  private final PhotonCamera backsidePhotonCamera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonCamera frontsidePhotonCamera, PhotonCamera backsidePhotonCamera) {
    this.frontsidePhotonCamera=frontsidePhotonCamera;
    this.backsidePhotonCamera=backsidePhotonCamera;

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    zeroHeading();

    try{

      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure( 
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
             new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    //Cam mounted facing forward and to right side, 22.75" (0.58m) from robot center in X and Y dir, and 8.5" (0.22m) Up from ground
    robotToFrontsidePhotonCamera = new Transform3d(new Translation3d(0.58, 0.58, 0.22), new Rotation3d(0,0,0));
    //Cam mounted facing backwardand to left side, 22.75" (0.58m) from robot center in X and Y dir, and 8.5" (0.22m) Up from ground
    robotToBacksidePhotonCamera = new Transform3d(new Translation3d(-0.58, -0.58, 0.22), new Rotation3d(0,0,0));

    // Construct PhotonPoseEstimator
    photonPoseEstimatorFrontsideCam = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontsidePhotonCamera);
    photonPoseEstimatorFrontsideCam.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonPoseEstimatorBacksideCam = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontsidePhotonCamera);
    photonPoseEstimatorBacksideCam.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFrontsideCam() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : frontsidePhotonCamera.getAllUnreadResults()) {
        visionEst = photonPoseEstimatorFrontsideCam.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBacksideCam() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : backsidePhotonCamera.getAllUnreadResults()) {
        visionEst = photonPoseEstimatorFrontsideCam.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
          Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

      if (estimatedPose.isEmpty()) {
          // No pose input. Default to single-tag std devs
          curStdDevs = kSingleTagStdDevs;

      } else {
          // Pose present. Start running Heuristic
          var estStdDevs = kSingleTagStdDevs;
          int numTags = 0;
          double avgDist = 0;

          // Precalculation - see how many tags we found, and calculate an average-distance metric
          for (var tgt : targets) {
              var tagPose = photonPoseEstimatorFrontsideCam.getFieldTags().getTagPose(tgt.getFiducialId());
              if (tagPose.isEmpty()) continue;
              numTags++;
              avgDist +=
                      tagPose
                              .get()
                              .toPose2d()
                              .getTranslation()
                              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          }

          if (numTags == 0) {
              // No tags visible. Default to single-tag std devs
              curStdDevs = kSingleTagStdDevs;
          } else {
              // One or more tags visible, run the full heuristic.
              avgDist /= numTags;
              // Decrease std devs if multiple targets are visible
              if (numTags > 1) estStdDevs = kMultiTagStdDevs;
              // Increase std devs based on (average) distance
              if (numTags == 1 && avgDist > 4)
                  estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
              else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
              curStdDevs = estStdDevs;
          }
      }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
      return curStdDevs;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        //Rotation2d.fromDegrees(-1*m_gyro.getYaw()),
        Rotation2d.fromDegrees((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Correct pose estimate with vision measurements
    if (DriveConstants.usePhotonPoseEstimator) {
      var frontsideVisionEst = getEstimatedGlobalPoseFrontsideCam();
      var backsideVisionEst = getEstimatedGlobalPoseBacksideCam();
      frontsideVisionEst.ifPresent(
              est -> {
                  // Change our trust in the measurement based on the tags we can see
                  var estStdDevs = getEstimationStdDevs();
                  m_poseEstimator.addVisionMeasurement(
                          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
              });
      backsideVisionEst.ifPresent(
              est -> {
                  // Change our trust in the measurement based on the tags we can see
                  var estStdDevs = getEstimationStdDevs();
                  m_poseEstimator.addVisionMeasurement(
                          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
      }

    SmartDashboard.putData("Field", m_field);  
    //SmartDashboard.putNumber("Gyro Heading: ", getHeading()); 
    SmartDashboard.putNumber("Gyro Yaw: ", ((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getYaw())); 
    SmartDashboard.putNumber("Gyro Angle: ", ((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle())); 
 
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();

  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        //Rotation2d.fromDegrees(-1*m_gyro.getYaw()),
        Rotation2d.fromDegrees((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionalSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
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

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
         fieldRelative
             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle()))
             : 
            new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
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
    m_gyro.reset();
  }

  public Command zeroHeadingCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          zeroHeading();
        });
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    //return Rotation2d.fromDegrees(-1*m_gyro.getYaw()).getDegrees();
    return Rotation2d.fromDegrees((DriveConstants.kGyroReversed ? -1.0 : 1.0)*m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Adjust heading of the robot such that it is facing the Drive Station. */
   public void robotFacingDriveStation() {
     m_gyro.setAngleAdjustment(180.0);
   }

}
