// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.EndEffectorState;
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

public class EndEffectorSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMaxConfig leftMotorConfig;
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder leftEncoder;

  private SparkMax rightMotor;
  private SparkMaxConfig rightMotorConfig;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder rightEncoder;

  private LaserCan LCReefside;
  private LaserCan LCHopperside;

  private double leftTargetVelocity;
  private double rightTargetVelocity;

  private EndEffectorState endEffectorState;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {

    endEffectorState =  EndEffectorState.UNKNOWN;

    leftTargetVelocity=0;
    rightTargetVelocity=0;

    leftMotor = new SparkMax(EndEffectorConstants.kLeftEndEfMotorCanID, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();

    rightMotor = new SparkMax(EndEffectorConstants.kRightEndEfMotorCanID, MotorType.kBrushless);
    rightClosedLoopController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();

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
      .outputRange(EndEffectorConstants.kMinOutRange, EndEffectorConstants.kMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(EndEffectorConstants.kMinOutRange, EndEffectorConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);
    
    rightMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(EndEffectorConstants.kMinOutRange, EndEffectorConstants.kMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1)
      .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      .outputRange(EndEffectorConstants.kMinOutRange, EndEffectorConstants.kMaxOutRange, ClosedLoopSlot.kSlot1);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    LCReefside = new LaserCan(EndEffectorConstants.kLCReefsideCanID);
    try {
        LCReefside.setRangingMode(LaserCan.RangingMode.SHORT);
        LCReefside.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 12, 4, 4)); // 8, 8, 16, 16
        LCReefside.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }
    LCHopperside = new LaserCan(EndEffectorConstants.kLCHoppersideCanID);
    try {
        LCHopperside.setRangingMode(LaserCan.RangingMode.SHORT);
        LCHopperside.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        LCHopperside.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("ENDE Left Target Vel", 0);
    SmartDashboard.setDefaultNumber("ENDE Right Target Vel", 0);
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

  public void setTargetVelocity(double leftTargetVelocity, double rightTargetVelocity) {
    setLeftTargetVelocity(leftTargetVelocity);
    setRightTargetVelocity(rightTargetVelocity);
  }

  public void setLeftTargetVelocity(double leftTargetVelocity) {
    this.leftTargetVelocity=leftTargetVelocity;
  }
  public void setRightTargetVelocity(double rightTargetVelocity) {
    this.rightTargetVelocity=rightTargetVelocity;
  }

  public boolean reefDetected() {
    return false;
  }

  public int getReefsideDistanceMM() {
    LaserCan.Measurement lcReefsideMeasurement = LCReefside.getMeasurement();
    return lcReefsideMeasurement.distance_mm;
  }

  public int getHoppersideDistanceMM() {
    LaserCan.Measurement lcHoppersideMeasurement = LCHopperside.getMeasurement();
    return lcHoppersideMeasurement.distance_mm;
  }

  public EndEffectorState getEndEffectorState() {
    return this.endEffectorState;
  }

  public void setEndEffectorState(EndEffectorState endEffectorState) {
    this.endEffectorState =  endEffectorState;
  }

  public boolean isCoralLoaded() {
    if (this.endEffectorState==EndEffectorState.CORAL_LOADED)
      return true;
    else
      return false;
  }

  public boolean isCoralNotLoaded() {
    if (this.endEffectorState!=EndEffectorState.CORAL_LOADED)
      return true;
    else
      return false;
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

    if (EndEffectorConstants.kLeftTargetVelocityFromDashboard)
      leftTargetVelocity = SmartDashboard.getNumber("ENDE Left Target Vel", 0);
    leftClosedLoopController.setReference(leftTargetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    SmartDashboard.putNumber("ENDE Left Actual Vel", leftEncoder.getVelocity());

    if (EndEffectorConstants.kRightTargetVelocityFromDashboard)
      rightTargetVelocity = SmartDashboard.getNumber("ENDE Right Target Vel", 0);
    rightClosedLoopController.setReference(rightTargetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    SmartDashboard.putNumber("ENDE Right Actual Vel", rightEncoder.getVelocity());

    SmartDashboard.putNumber("ENDE LCReefside Dist (mm)", getReefsideDistanceMM());

    SmartDashboard.putNumber("ENDE LCHopperside Dist (mm)", getHoppersideDistanceMM());

    SmartDashboard.putBoolean("ENDE Coral Loaded", isCoralLoaded());
    SmartDashboard.putBoolean("ENDE Coral NOT Loaded", isCoralNotLoaded());

  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
