package frc.robot.commands;

import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;

public class ReverseIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem m_subsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private int isFinishedDelayCountInMs;

  public ReverseIntakeCommand(EndEffectorSubsystem subsystem) {
    m_subsystem = subsystem;
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
    m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorReverseIntakeTargetVelocity, EndEffectorConstants.kRightMotorReverseIntakeTargetVelocity);

    isFinishedDelayCountInMs+=20; // Adding 20ms which is how often execute() is called.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTargetVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinishedDelayCountInMs>EndEffectorConstants.kReveseIntakeCommandRuntimeInMs)
      return true;
    else
      return false;
  }
}
