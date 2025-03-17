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
  private final PhotonCamera backsidePhotonCamera;

  private final double CAMERA_HEIGHT_IN_METERS = 0.22;  // FIXME: 8.5" (0.22m) Up from ground, need to confirm
  private final double TARGET_HEIGHT_IN_METERS = 0.235; 
  private final double CAMERA_PITCH_IN_DEGREES = 20;   // FIXME: Camera pitch in degrees, positive up, need to confirm

  private final double VISION_DESIRED_ANGLE_deg = 155.0; // FIXME: Based on testing, 155 degrees is when Tag is centered
  private final double VISION_DESIRED_ANGLE_deg_tolerance = 5.0; 
  private final double VISION_DESIRED_RANGE_m = 0.5;     // FIXME: Based on testing, Tag is 0.5 meters away when at the reef.
  private final double VISION_TURN_kP = 0.065;
  private final double VISION_XSPEED_kP = 0.25;
  private final double FIXED_XSPEED = 0.3;  // FIXME: Could potentially be higher

  private final boolean usePhotonCamera = false;
  private boolean targetVisible = false;
  private double targetYaw = 0.0;
  private double targetRange = 0.0;
  private double xSpeed;
  private double rotateSpeed;
  private boolean useFixedxSpeed = true;


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
    targetVisible = false;
    targetYaw = 0.0;
    targetRange = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetYawOffset;

    // By default the command will not move robot
    targetVisible = false;
    rotateSpeed=0.0;
    xSpeed=0.0;

    if (usePhotonCamera) {
      // Read in relevant data from the Camera
      var results = backsidePhotonCamera.getAllUnreadResults();

      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera    
            for (var target : result.getTargets()) {
                // ONLY looking for those at the Coral Stations (as documented in Game Manual)
                if ((target.getFiducialId() == 1) || (target.getFiducialId() == 2) ||  
                    (target.getFiducialId() == 12) || (target.getFiducialId() == 13)) {

                    targetYaw = target.getYaw();
                    targetRange =
                            PhotonUtils.calculateDistanceToTargetMeters(
                              CAMERA_HEIGHT_IN_METERS,
                              TARGET_HEIGHT_IN_METERS, 
                              Units.degreesToRadians(CAMERA_PITCH_IN_DEGREES), // Camera pitch in degrees,positive up
                              Units.degreesToRadians(target.getPitch()));

                    targetVisible = true;
                    //SmartDashboard.putBoolean("Coral Station Tag Detected", true);
                }
            }
        }
      }
      else {
        targetVisible = false;
        //SmartDashboard.putBoolean("Coral Station Tag Detected", false);
      }

      if (targetVisible) {
        targetYawOffset=Math.abs(VISION_DESIRED_ANGLE_deg-targetYaw);
        if (targetYawOffset>VISION_DESIRED_ANGLE_deg_tolerance) {
          rotateSpeed = targetYawOffset * VISION_TURN_kP;
          if (targetYaw<0)
            rotateSpeed = -1.0 * rotateSpeed;
        }
        
        if (useFixedxSpeed) {
          xSpeed = -1.0 * FIXED_XSPEED;
          }
        else {
          if (targetRange>VISION_DESIRED_RANGE_m) {
            xSpeed = -1.0 * (targetRange-VISION_DESIRED_RANGE_m) * VISION_XSPEED_kP;
          }
        }

        swerveDriveTrain.drive(
          -MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband),
          0,
          -MathUtil.applyDeadband(rotateSpeed, OIConstants.kDriveDeadband),
          false, 
          true);
       }
       else {
         swerveDriveTrain.drive(0, 0, 0, true, true);
       }
    }
  
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