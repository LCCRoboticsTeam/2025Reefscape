// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AlgaeArmState;
import frc.robot.subsystems.AlgaeArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeArmSubsystem m_subsystem;
  private AlgaeArmState m_algaeArmState;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeArmCommand(AlgaeArmSubsystem subsystem, AlgaeArmState algaeArmState) {
    m_subsystem = subsystem;
    m_algaeArmState = algaeArmState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setArmTargetPosition(m_algaeArmState.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      m_subsystem.setAlgaeArmState(m_algaeArmState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.getArmActualPosition()==m_algaeArmState.getPosition())
      return true;
    else
      return false;
  }
}
