// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveToCenterCoralStationCommand extends Command {

  private final DriveSubsystem swerveDriveTrain;
  private double xSpeed;
  private double rotateSpeed;
  private final PhotonCamera backsidePhotonCamera;
  private final boolean usePhotonCamera = true;

  private final double CAMERA_HEIGHT_IN_METERS = 0.22;  // FIXME: 8.5" (0.22m) Up from ground, need to confirm
  private final double TARGET_HEIGHT_IN_METERS = 0.235; 
  private final double CAMERA_PITCH_IN_DEGREES = 0.0;   // FIXME: Camera pitch in degrees, positive up, need to confirm

  private final double VISION_DES_ANGLE_deg = 155.0; // FIXME: Need to update based on testing
  private final double VISION_DES_RANGE_m = 0.5;     // FIXME: Need to update based on testing
  private final double VISION_TURN_kP = 0.0000001; // was 0.1
  private final double VISION_XSPEED_kP = 0.01;

  double targetYaw = 0.0;
  double targetRange = 0.0;

  /** Creates a new SwerveControllerDrive. */
  public SwerveToCenterCoralStationCommand(DriveSubsystem swerveDriveTrain, PhotonCamera backsidePhotonCamera) {
    this.swerveDriveTrain=swerveDriveTrain;
    this.backsidePhotonCamera = backsidePhotonCamera;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Coral Station Tag Detected", false);
    SmartDashboard.putNumber("targetYaw", targetYaw);
    SmartDashboard.putNumber("targetRange", targetRange);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Read in relevant data from the Camera
    boolean targetVisible = false;

    rotateSpeed=0.0;
    xSpeed=0.0;

    if (usePhotonCamera) {
      var results = backsidePhotonCamera.getAllUnreadResults();

      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.

          var result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera    
              for (var target : result.getTargets()) {
                  // ONLY looking for those at the Coral Stations (as documented in Game Manual)
                  if ((target.getFiducialId() == 1) || (target.getFiducialId() == 2) && 
                      (target.getFiducialId() == 12) && (target.getFiducialId() == 13)) {
                      targetYaw = target.getYaw();

                      targetRange =
                              PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_IN_METERS,
                                TARGET_HEIGHT_IN_METERS, 
                                Units.degreesToRadians(CAMERA_PITCH_IN_DEGREES), // Camera pitch in degrees,positive up
                                Units.degreesToRadians(target.getPitch()));

                      targetVisible = true;
                      SmartDashboard.putBoolean("Coral Station Tag Detected", true);
                  }
              }
          }
      }
      else {
        SmartDashboard.putBoolean("Coral Station Tag Detected", false);
      }

      if (targetVisible) {
        // Based on testing, 155 degrees is when Tag is centered, so offset the targetYaw in formula below by
        // that amount
        rotateSpeed = 1.0 * Math.abs(VISION_DES_ANGLE_deg-targetYaw) * VISION_TURN_kP * DriveConstants.kMaxAngularSpeed;
        //if ((VISION_DES_ANGLE_deg-targetYaw) <0)
          //rotateSpeed=-1.0*rotateSpeed;
        xSpeed = Math.abs(targetRange - VISION_DES_RANGE_m) * VISION_XSPEED_kP * DriveConstants.kMaxSpeedMetersPerSecond;
       }
    }

    SmartDashboard.putNumber("targetYaw", targetYaw);
    SmartDashboard.putNumber("targetRange", targetRange);

    swerveDriveTrain.drive(
                -MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband),
                0,
                -MathUtil.applyDeadband(rotateSpeed, OIConstants.kDriveDeadband),
                false, 
                true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command runs will xbox trigger is pressed, so it runs until it is interrupted
    return false;
  }

}