// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
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

public class AlgaeSubsystem extends SubsystemBase {

  private SparkMax armMotor;
  private SparkMaxConfig armMotorConfig;
  private SparkClosedLoopController armClosedLoopController;
  private RelativeEncoder armEncoder;

  private SparkMax wheelMotor;
  private SparkMaxConfig wheelMotorConfig;
  private SparkClosedLoopController wheelClosedLoopController;
  private RelativeEncoder wheelEncoder;

  private double armTargetPosition;
  private double wheelTargetVelocity;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {

    armTargetPosition=0;
    wheelTargetVelocity=0;

    armMotor = new SparkMax(AlgaeConstants.kArmAlgaeMotorCanID, MotorType.kBrushless);
    armClosedLoopController = armMotor.getClosedLoopController();
    armEncoder = armMotor.getEncoder();

    wheelMotor = new SparkMax(AlgaeConstants.kWheelAlgaeMotorCanID, MotorType.kBrushless);
    wheelClosedLoopController = wheelMotor.getClosedLoopController();
    wheelEncoder = wheelMotor.getEncoder();

    armMotorConfig = new SparkMaxConfig();

    wheelMotorConfig = new SparkMaxConfig();

    armMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);
    
    wheelMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      // Set up PID closed loop
    armMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(AlgaeConstants.kMinOutRange, AlgaeConstants.kMaxOutRange)
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(AlgaeConstants.kMinOutRange, AlgaeConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);
  
  wheelMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(AlgaeConstants.kMinOutRange, AlgaeConstants.kMaxOutRange)
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(AlgaeConstants.kMinOutRange, AlgaeConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);

  armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, 
    PersistMode.kNoPersistParameters);
  
  wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, 
    PersistMode.kNoPersistParameters);
  
  SmartDashboard.setDefaultNumber("ALGAE Arm Target Position", 0);
  SmartDashboard.setDefaultNumber("ALGAE Wheel Target Velocity", 0);
  
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armTargetPosition = SmartDashboard.getNumber("ENDE Arm Target Position", 0);
    armClosedLoopController.setReference(armTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("ENDE Arm Actual Position", armEncoder.getVelocity());

    wheelTargetVelocity = SmartDashboard.getNumber("ENDE Wheel Target Velocity", 0);
    wheelClosedLoopController.setReference(wheelTargetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    SmartDashboard.putNumber("ENDE Wheel Actual Velocity", wheelEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
