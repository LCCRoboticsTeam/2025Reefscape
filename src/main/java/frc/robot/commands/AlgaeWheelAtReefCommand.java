// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeWheelState;
import frc.robot.subsystems.AlgaeWheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeWheelAtReefCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeWheelSubsystem m_subsystem;
  private boolean immediateFinish;
  private boolean abortCommand;
  private boolean gotAlgae;
  private int isFinishedDelayCountInMs;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeWheelAtReefCommand(AlgaeWheelSubsystem subsystem, boolean immediateFinish) {
    m_subsystem = subsystem;
    this.immediateFinish = immediateFinish;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinishedDelayCountInMs=0;
    abortCommand=false;
    gotAlgae=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefTargetVelocity);  
      
    isFinishedDelayCountInMs+=20; // Adding 20ms which is how often execute() is called.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (abortCommand) {
      m_subsystem.setWheelTargetVelocity(0);
    }
    else if (gotAlgae) {
      m_subsystem.setAlgaeWheelState(AlgaeWheelState.ALGAE_LOADED_FROM_REEF);
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefHoldingTargetVelocity);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (immediateFinish) {
      return true;
    }
    else {
      // Detect that we have Algae if wheel velocity slows down below a threshold
      if (m_subsystem.getWheelActualVelocity()<AlgaeConstants.kAlgaeWheelAtReefHoldingVelocityThreshold) {
        gotAlgae=true;
        return true;
        }
      else {
        if (isFinishedDelayCountInMs>AlgaeConstants.kAlgaeWheelAtReefCommandMaxRuntimeInMs) {
          abortCommand=true;
          return true;
        }
        else {
          return false;
        } 
      }
    }
  }
}
