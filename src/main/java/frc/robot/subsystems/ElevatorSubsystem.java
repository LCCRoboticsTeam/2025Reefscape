// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMaxConfig leftMotorConfig;
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder leftEncoder;
  //private SparkLimitSwitch m_forwardLimit;
  //private SparkLimitSwitch m_reverseLimit;

  private SparkMax rightMotor;

  private ElevatorState elevatorState;

  double targetPosition;

  /** Creates a new EndEffectorSubsystem. */
  public ElevatorSubsystem() {

    elevatorState = ElevatorState.UNKNOWN;
    targetPosition = 0;

    // Left Motor is the leader
    leftMotor = new SparkMax(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);

    leftMotorConfig = new SparkMaxConfig();

    leftMotorConfig.idleMode(IdleMode.kBrake);

    leftMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);  

    //m_forwardLimit = leftMotor.getForwardLimitSwitch();
    //m_reverseLimit = leftMotor.getReverseLimitSwitch();

    //m_forwardLimit. enableLimitSwitch(false);
    //m_reverseLimit.enableLimitSwitch(false);
    //SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
    //SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
   
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
    
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);

    // Right Motor is the follower
    rightMotor = new SparkMax(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);

    rightMotor.configure(leftMotorConfig.follow(leftMotor, true), ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("ELEV Target Pos", 0);

  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition=targetPosition;
  }

  public void setElevatorState(ElevatorState state){
    this.elevatorState = state;
  }

  public ElevatorState getElevatorState(){
    return this.elevatorState;
  }

  public double getLeftActualPosition() {
   return leftEncoder.getPosition();
  }

   public void resetPosition() {
    // Reset the encoder position to 0
    leftEncoder.setPosition(0);
  }

  public boolean isElevatorAtP1() {
    if (this.elevatorState==ElevatorState.P1)
      return true;
    else
      return false;
  }
  public boolean isElevatorAtP1orP1p5() {
    if ((this.elevatorState==ElevatorState.P1) || (this.elevatorState==ElevatorState.P1p5))
      return true;
    else
      return false;
  }
  public boolean isElevatorNotAtP1() {
    if (this.elevatorState!=ElevatorState.P1)
      return true;
    else
      return false;
  }
  public boolean isElevatorNotAtP1orP1p5() {
    if ((this.elevatorState!=ElevatorState.P1) && (this.elevatorState!=ElevatorState.P1p5))
      return true;
    else
      return false;
  }
  public boolean isElevatorAtP4() {
    if (this.elevatorState==ElevatorState.P4)
      return true;
    else
      return false;
  }

  public Command resetPositionCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          resetPosition();
        });
  }

  public Command elevateToP2() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          setTargetPosition(ElevatorState.P2.getPosition()); setElevatorState(ElevatorState.P2);
        });
  }

  public Command elevateToP3() {
    return runOnce(
      () -> {
        /* one-time action goes here */
        setTargetPosition(ElevatorState.P3.getPosition()); setElevatorState(ElevatorState.P3);
      });
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (ElevatorConstants.kTargetPositionFromDashboard)
      targetPosition = SmartDashboard.getNumber("ELEV Target Pos", 0);

    // Since we reset to Postion 0, which is when the elevator is down,
    // we should NEVER allow a position that is negative
    if (targetPosition>=0) {
      leftClosedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    SmartDashboard.putNumber("ELEV Actual Pos", leftEncoder.getPosition());

    SmartDashboard.putNumber("ELEV Left Amps", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("ELEV Right Amps", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("ELEV Left DutyCycle", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("ELEV Right DutyCycle", rightMotor.getAppliedOutput());
    //SparkLimitSwitch forwardLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit FWD", forwardLimitSwitch.isPressed());
    //SparkLimitSwitch reverseLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit REV", reverseLimitSwitch.isPressed());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
