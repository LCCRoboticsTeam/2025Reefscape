package frc.robot.commands;

import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.EndEffectorState;
import frc.robot.Constants.PlaceCoralDirection;

public class PlaceCoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem m_subsystem;
  private int isFinishedDelayCountInMs;
  private PlaceCoralDirection placeCoralDirection;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public PlaceCoralCommand(EndEffectorSubsystem subsystem, PlaceCoralDirection placeCoralDirection) {
    m_subsystem = subsystem;
    this.placeCoralDirection = placeCoralDirection;
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
    if (placeCoralDirection==PlaceCoralDirection.PLACE_CORAL_RIGHT)
      m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorPlaceCoralRightTargetVelocity, EndEffectorConstants.kRightMotorPlaceCoralRightTargetVelocity);
    else if (placeCoralDirection==PlaceCoralDirection.PLACE_CORAL_LEFT)
      m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorPlaceCoralLeftTargetVelocity, EndEffectorConstants.kRightMotorPlaceCoralLeftTargetVelocity);
    else
    m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorPlaceCoralTargetVelocity, EndEffectorConstants.kRightMotorPlaceCoralTargetVelocity);

    isFinishedDelayCountInMs+=20; // Adding 20ms which is how often execute() is called.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTargetVelocity(0, 0);
    if (!interrupted)
        m_subsystem.setEndEffectorState(EndEffectorState.CORAL_FREE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinishedDelayCountInMs>EndEffectorConstants.kPlaceCoralCommandRuntimeInMs)
      return true;
    else
      return false;
  }
}
