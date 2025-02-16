// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.AlgaeWheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeWheelAtReefCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeWheelSubsystem m_subsystem;
  private int isFinishedDelayCountInMs;
  private int maxcount;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeWheelAtReefCommand(AlgaeWheelSubsystem subsystem, int maxcount) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.maxcount = maxcount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinishedDelayCountInMs=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefTargetVelocity);
    isFinishedDelayCountInMs+=20;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) 
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefTargetVelocity/8);
    else if (maxcount>2000)
      m_subsystem.setWheelTargetVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // FIXME: Will want to stop when we believe we are holding Algae, maybe check for a motor current increase??
    //        OR driver just releases button when they think they have it.
    if (isFinishedDelayCountInMs>maxcount)
      return true;
    else
      return false;
  }
}
