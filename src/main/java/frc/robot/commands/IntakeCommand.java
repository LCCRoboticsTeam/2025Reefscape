package frc.robot.commands;

import frc.robot.subsystems.EndEffectorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.EndEffectorState;

public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem m_subsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private boolean coralDetected;
  private int coreDetectedCount;

  public IntakeCommand(EndEffectorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralDetected = false;
    coreDetectedCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Slowdown once we know we have coral coming into the EndEffector so that it stops
    // at a more repeatable position when this command ends.
    if (coralDetected==true)
      m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorIntakeTargetVelocity/1.5, EndEffectorConstants.kRightMotorIntakeTargetVelocity/2);
    else
      m_subsystem.setTargetVelocity(EndEffectorConstants.kLeftMotorIntakeTargetVelocity, EndEffectorConstants.kRightMotorIntakeTargetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTargetVelocity(0, 0);
    if (!interrupted)
        m_subsystem.setEndEffectorState(EndEffectorState.CORAL_LOADED);

    SmartDashboard.putNumber("coreDetectedCount", coreDetectedCount);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.getHoppersideDistanceMM() < EndEffectorConstants.kCoralDetectedDistance) {
      coreDetectedCount++;
      if (coreDetectedCount>EndEffectorConstants.kCorelDetectedCountThreshold) {
        coralDetected=true;
      }
    } else if (coralDetected == true) {
        return true;
    }
    return false;
  }
}
