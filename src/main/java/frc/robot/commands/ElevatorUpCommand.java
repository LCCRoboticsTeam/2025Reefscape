// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorUpCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_subsystem;
  private final BooleanSupplier m_isCoralLoaded;
  private boolean m_forceToP1p5;

  ElevatorState m_elevatorState = ElevatorState.P1;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorUpCommand(ElevatorSubsystem subsystem, boolean forceToP1p5, BooleanSupplier isCoralLoaded) {
    m_subsystem = subsystem;
    m_forceToP1p5 = forceToP1p5;
    m_isCoralLoaded = isCoralLoaded;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_forceToP1p5) {
      m_elevatorState=ElevatorState.P1p5;
    }
    else {
      if (m_isCoralLoaded.getAsBoolean()) {
        if (m_subsystem.getElevatorState()==ElevatorState.P1)
          m_elevatorState=ElevatorState.P2;
        else {
          if (m_subsystem.getElevatorState()==ElevatorState.P2)
            m_elevatorState=ElevatorState.P3;
          else 
            m_elevatorState=ElevatorState.P4;
          }
      }
      else {
        if (m_subsystem.getElevatorState()==ElevatorState.P1)
          m_elevatorState=ElevatorState.P3;
        else 
          m_elevatorState=ElevatorState.P3p5;
      }
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setTargetPosition(m_elevatorState.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setElevatorState(m_elevatorState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
