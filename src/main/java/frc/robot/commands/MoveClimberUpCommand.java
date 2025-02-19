// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberState;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveClimberUpCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveClimberUpCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_subsystem.setServoAngle(ClimberConstants.kServoAngleToDisableRatchet);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setTargetPosition(ClimberConstants.kClimberPositionUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      m_subsystem.setClimberState(ClimberState.CLIMBER_UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.getActualPosition()==ClimberConstants.kClimberPositionUp)
      return true;
    else
      return false;
  }
}
