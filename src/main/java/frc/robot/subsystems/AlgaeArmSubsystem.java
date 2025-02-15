// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeArmState;
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

public class AlgaeArmSubsystem extends SubsystemBase {

  private SparkMax armMotor;
  private SparkMaxConfig armMotorConfig;
  private SparkClosedLoopController armClosedLoopController;
  private RelativeEncoder armEncoder;

  private double armTargetPosition;

  private AlgaeArmState algaeArmState;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeArmSubsystem() {

    armTargetPosition=0;

    algaeArmState=AlgaeArmState.UNKNOWN;

    armMotor = new SparkMax(AlgaeConstants.kArmAlgaeMotorCanID, MotorType.kBrushless);
    armClosedLoopController = armMotor.getClosedLoopController();
    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);

    armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode(IdleMode.kBrake);

    armMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    // Set up PID closed loop
    armMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(AlgaeConstants.kAlgaeArmMinOutRange, AlgaeConstants.kAlgaeArmMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(AlgaeConstants.kAlgaeArmMinOutRange, AlgaeConstants.kAlgaeArmMaxOutRange, ClosedLoopSlot.kSlot1);
  
    armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);

    SmartDashboard.setDefaultNumber("ALGE Arm Target Pos", 0);
  
  }

  public void setArmMotorIdleModeToCoast() {
    armMotorConfig.idleMode(IdleMode.kCoast);
  }

  public Command setArmMotorIdleModeToBrakeCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          armMotorConfig.idleMode(IdleMode.kBrake);
        });
  }

  public Command setArmMotorIdleModeToCoastCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          armMotorConfig.idleMode(IdleMode.kCoast);
        });
  }

  public void setArmTargetPosition(double armTargetPosition) {
    // POSITIVE Value -> Arm UP
    // NEGATIVE Value -> Arm DOWN
    this.armTargetPosition=armTargetPosition;
  }

  public double getArmActualPosition() {
   return armEncoder.getPosition();
  }

  public void resetArmPosition() {
    // Reset the encoder position to 0
    armEncoder.setPosition(0);
 }

  public AlgaeArmState getAlgaeArmState() {
    return this.algaeArmState;
  }

  public void setAlgaeArmState(AlgaeArmState algaeArmState) {
    this.algaeArmState =  algaeArmState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (AlgaeConstants.kArmTargetPositionFromDashboard)
      armTargetPosition = SmartDashboard.getNumber("ALGE Arm Target Pos", 0);
    armClosedLoopController.setReference(armTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("ALGE Arm Actual Pos", armEncoder.getPosition());

    SmartDashboard.putNumber("ALGE Arm Amps", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("ALGE Arm DutyCycle", armMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
