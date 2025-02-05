// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMaxConfig leftMotorConfig;
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder leftEncoder;

  private SparkMax rightMotor;
  private SparkMaxConfig rightMotorConfig;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder rightEncoder;

  double targetPosition;

  /** Creates a new EndEffectorSubsystem. */
  public ElevatorSubsystem() {

    targetPosition = 0;

    leftMotor = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);

    rightMotor = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
    rightClosedLoopController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();
    rightEncoder.setPosition(0);

    leftMotorConfig = new SparkMaxConfig();

    rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    
    rightMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    
    // Set up PID closed loop
    leftMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(ElevatorConstants.kMinOutRange, ElevatorConstants.kMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(ElevatorConstants.kMinOutRange, ElevatorConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);
    
    rightMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(ElevatorConstants.kMinOutRange, ElevatorConstants.kMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(ElevatorConstants.kMinOutRange, ElevatorConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Left Target Position", 0);
    SmartDashboard.setDefaultBoolean("Left Reset Encoder", false);

    SmartDashboard.setDefaultNumber("Right Target Position", 0);
    SmartDashboard.setDefaultBoolean("Right Reset Encoder", false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition=targetPosition;
  }

  public double getLeftActualPosition() {
   return leftEncoder.getPosition();
  }

  public double getRightActualPosition() {
    return rightEncoder.getPosition();
   }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetPosition = SmartDashboard.getNumber("Elevator Target Position", 0);

    //FIXME: Need to confirm which one is negative, left or right, starting with right
    leftClosedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightClosedLoopController.setReference(-1*targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("Elevator Left Actual Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Right Actual Position", rightEncoder.getPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
