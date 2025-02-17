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
  private boolean immediateFinish;
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
    //this.maxcount = maxcount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefTargetVelocity);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!immediateFinish)
      m_subsystem.setWheelTargetVelocity(AlgaeConstants.kAlgaeWheelAtReefHoldingTargetVelocity);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((immediateFinish) || (m_subsystem.getWheelActualVelocity()<10)) // So nearly stopped indicating we have algae
      return true;
    else
      return false;
  }
}
