// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveRotateCommand extends Command {

  private final DriveSubsystem swerveDriveTrain;
  private int isFinishedDelayCountInMs;
  double rotateSpeed;

  /** Creates a new SwerveControllerDrive. */
  public SwerveRotateCommand(DriveSubsystem swerveDriveTrain, double rotateSpeed) {
    this.swerveDriveTrain=swerveDriveTrain;
    this.rotateSpeed=rotateSpeed;
      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinishedDelayCountInMs=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveTrain.drive(
                0,
                0,
                -MathUtil.applyDeadband(rotateSpeed, OIConstants.kDriveDeadband),
                false, 
                true);

     isFinishedDelayCountInMs+=20; // Adding 20ms which is how often execute() is called.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinishedDelayCountInMs>DriveConstants.kSwerveRotateCommandRuntimeInMs)
      return true;
    else
      return false;
  }

}