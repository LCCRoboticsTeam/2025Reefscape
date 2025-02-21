// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveGamepadDriveCommand extends Command {

  private final DriveSubsystem swerveDriveTrain;
  private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotateSpeedSupplier;
  private final BooleanSupplier slowMode;
  private final BooleanSupplier getLeftStickButton;
  private final PhotonCamera frontsidePhotonCamera;
  //private PhotonCamera backsidePhotonCamera;

  private final double VISION_TURN_kP = 0.01;

  /** Creates a new SwerveControllerDrive. */
  public SwerveGamepadDriveCommand(DriveSubsystem swerveDriveTrain, DoubleSupplier ySpeedSupplier,
      DoubleSupplier xSpeedSupplier, DoubleSupplier rotateSpeedSupplier, BooleanSupplier slowMode, 
      BooleanSupplier getLeftStickButton, PhotonCamera frontsidePhotonCamera) {
    this.swerveDriveTrain = swerveDriveTrain;
    this.ySpeedSupplier = ySpeedSupplier;
    this.xSpeedSupplier = xSpeedSupplier;
    this.rotateSpeedSupplier = rotateSpeedSupplier;
    this.slowMode = slowMode;
    this.getLeftStickButton = getLeftStickButton;
    this.frontsidePhotonCamera = frontsidePhotonCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //swerveDriveTrain.zeroHeading();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedSupplier.getAsDouble();
    double ySpeed = ySpeedSupplier.getAsDouble();
    double rotateSpeed = rotateSpeedSupplier.getAsDouble();

    if (slowMode.getAsBoolean()) {
      if (xSpeed<0)
        xSpeed = Math.max(xSpeed, -1*DriveConstants.kSwerveSlideSpeed);
      else
        xSpeed = Math.min(xSpeed, DriveConstants.kSwerveSlideSpeed);
      if (ySpeed<0)
        ySpeed = Math.max(ySpeed, -1*DriveConstants.kSwerveSlideSpeed);
      else
        ySpeed = Math.min(ySpeed, DriveConstants.kSwerveSlideSpeed);
      if (rotateSpeed<0)
        rotateSpeed = Math.max(rotateSpeed, -1*DriveConstants.kSwerveSlideSpeed);
      else
        rotateSpeed = Math.min(rotateSpeed, DriveConstants.kSwerveSlideSpeed);
    }

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = frontsidePhotonCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                // ONLY looking for those at the REEF or PROCESSOR, so check 
                // that it not one of the others (as documented in Game Manual
                if ((target.getFiducialId() != 1) && (target.getFiducialId() != 2) &&
                    (target.getFiducialId() != 4) && (target.getFiducialId() != 5) &&
                    (target.getFiducialId() != 12) && (target.getFiducialId() != 13) &&
                    (target.getFiducialId() != 14) && (target.getFiducialId() != 15)) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        }
    }

    if (getLeftStickButton.getAsBoolean() && targetVisible) {
      //  Choosing 1/80 degrees max Yah gives 0.0125
      //rotateSpeed = -1.0 * targetYaw * 0.0125;
      rotateSpeed = -1.0 * targetYaw * VISION_TURN_kP * DriveConstants.kMaxAngularSpeed;
    }

    swerveDriveTrain.drive(
                -MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(ySpeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rotateSpeed, OIConstants.kDriveDeadband),
                true,
                true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.drive(0, 0, 0, interrupted, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}