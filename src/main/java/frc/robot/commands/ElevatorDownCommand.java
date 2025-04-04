// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorDownCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_subsystem;
  ElevatorState m_ElevatorState = ElevatorState.P1;
  boolean m_forceToP1;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorDownCommand(ElevatorSubsystem subsystem, boolean forceToP1) {
    m_subsystem = subsystem;
    m_forceToP1 = forceToP1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((m_forceToP1) || 
        (m_subsystem.getElevatorState()==ElevatorState.P2) || 
        (m_subsystem.getElevatorState()==ElevatorState.P1p5) ||
        (m_subsystem.getElevatorState()==ElevatorState.P1) || (m_subsystem.getElevatorState()==ElevatorState.P3p5))
      m_ElevatorState=ElevatorState.P1;
    else {
      if (m_subsystem.getElevatorState()==ElevatorState.P3)
        m_ElevatorState=ElevatorState.P2;
      else // At P4 or P3p5
        m_ElevatorState=ElevatorState.P3; 
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setTargetPosition(m_ElevatorState.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setElevatorState(m_ElevatorState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
