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

  private PhotonCamera frontsidePhotonCamera;
  private PhotonCamera backsidePhotonCamera;

  /** Creates a new SwerveControllerDrive. */
  public SwerveGamepadDriveCommand(DriveSubsystem swerveDriveTrain, DoubleSupplier ySpeedSupplier,
      DoubleSupplier xSpeedSupplier, DoubleSupplier rotateSpeedSupplier, BooleanSupplier slowMode) {
    this.swerveDriveTrain = swerveDriveTrain;
    this.ySpeedSupplier = ySpeedSupplier;
    this.xSpeedSupplier = xSpeedSupplier;
    this.rotateSpeedSupplier = rotateSpeedSupplier;
    this.slowMode = slowMode;
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

  public void setfrontsidePhotonCamera(PhotonCamera camera) {
    this.frontsidePhotonCamera = camera;
  }

  public void setbacksidePhotonCamera(PhotonCamera camera) {
    this.backsidePhotonCamera = camera;
  }

}