// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveSlideCommand extends Command {

  private final DriveSubsystem swerveDriveTrain;
  boolean rightDirection;
  double ySpeed;

  /** Creates a new SwerveControllerDrive. */
  public SwerveSlideCommand(DriveSubsystem swerveDriveTrain, boolean rightDirection, double ySpeed) {
    this.swerveDriveTrain = swerveDriveTrain;
    this.rightDirection = rightDirection;
    if (rightDirection)
      this.ySpeed=ySpeed;
    else
      this.ySpeed=-ySpeed;
      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveTrain.drive(
                0,
                -MathUtil.applyDeadband(ySpeed, OIConstants.kDriveDeadband),
                0,
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
    return false;
  }

}