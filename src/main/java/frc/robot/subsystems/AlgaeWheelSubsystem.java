// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class AlgaeWheelSubsystem extends SubsystemBase {

  private SparkMax wheelMotor;
  private SparkMaxConfig wheelMotorConfig;
  private SparkClosedLoopController wheelClosedLoopController;
  private RelativeEncoder wheelEncoder;

  private double wheelTargetVelocity;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeWheelSubsystem() {

    wheelTargetVelocity=0;

    wheelMotor = new SparkMax(AlgaeConstants.kWheelAlgaeMotorCanID, MotorType.kBrushless);
    wheelClosedLoopController = wheelMotor.getClosedLoopController();
    wheelEncoder = wheelMotor.getEncoder();

    wheelMotorConfig = new SparkMaxConfig();
    wheelMotorConfig.idleMode(IdleMode.kBrake);

    wheelMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    // Set up PID closed loop
    wheelMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(AlgaeConstants.kAlgaeWheelMinOutRange, AlgaeConstants.kAlgaeWheelMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(AlgaeConstants.kAlgaeWheelMinOutRange, AlgaeConstants.kAlgaeWheelMaxOutRange, ClosedLoopSlot.kSlot1);

    wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
  
    SmartDashboard.setDefaultNumber("ALGE Wheel Target Vel", 0);
  
  }

  public void setWheelTargetVelocity(double wheelTargetVelocity) {
    // POSITIVE Value -> Wheels Spin UP/IN when facing reef, our OUT to processor
    // NEGATIVE Value -> Wheels Spin DOWN/in when picking from ground
    this.wheelTargetVelocity=wheelTargetVelocity;
  }

  public double getWheelActualVelocity() {
   return wheelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (AlgaeConstants.kWheelTargetVelocityFromDashboard)
      wheelTargetVelocity = SmartDashboard.getNumber("ALGAE Wheel Target Vel", 0);
    wheelClosedLoopController.setReference(wheelTargetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    SmartDashboard.putNumber("ALGE Wheel Actual Vel", wheelEncoder.getVelocity());

    SmartDashboard.putNumber("ALGE Wheel Amps", wheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("ALGE Wheel DutyCycle", wheelMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
