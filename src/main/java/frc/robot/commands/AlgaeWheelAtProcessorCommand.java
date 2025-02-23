// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeWheelState;
import frc.robot.subsystems.AlgaeWheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeWheelAtProcessorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeWheelSubsystem m_subsystem;
  private boolean m_algaeFromReef;
  private int isFinishedDelayCountInMs;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeWheelAtProcessorCommand(AlgaeWheelSubsystem subsystem, boolean algaeFromReef) {
    m_subsystem = subsystem;
    m_algaeFromReef = algaeFromReef;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinishedDelayCountInMs=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_algaeFromReef)
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtProcessorReefAlgaeTargetVelocity);
    else
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtProcessorGroundAlgaeTargetVelocity);

    isFinishedDelayCountInMs+=20; // Adding 20ms which is how often execute() is called.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setWheelTargetVelocity(0);
    m_subsystem.setAlgaeWheelState(AlgaeWheelState.ALGAE_FREE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinishedDelayCountInMs>AlgaeConstants.kAlgaeWheelAtProcessorCommandRuntimeInMs)
      return true;
    else
      return false;
  }
}
