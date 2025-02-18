// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // -------------------- ENDEFFECTOR --------------------
  public static final class EndEffectorConstants {
    public static final int kRightEndEfMotorCanID = 5;
    public static final int kLeftEndEfMotorCanID = 6;
    public static final int kLCHoppersideCanID = 7;
    public static final int kLCReefsideCanID = 8;

    public static final double kMaxOutRange = 0.8;
    public static final double kMinOutRange = -0.8;

    // NOTE: For intaking and placing coral, Motor direction MUST BE:
    //         LeftMotor -> Positive
    //         RightMotor -> Negative
    public static final double kLeftMotorIntakeTargetVelocity = 2000;
    public static final double kRightMotorIntakeTargetVelocity = -2000;
    public static final double kLeftMotorIntakeCoralDetectedTargetVelocity = 1200;
    public static final double kRightMotorIntakeCoralDetectedTargetVelocity = -1200;
    // PLACE STRAIGHT
    public static final double kLeftMotorPlaceCoralTargetVelocity = 3500;
    public static final double kRightMotorPlaceCoralTargetVelocity = -3500;
    // PLACE L4
    public static final double kLeftMotorPlaceCoralL4TargetVelocity = 3500;
    public static final double kRightMotorPlaceCoralL4TargetVelocity = -3500;
    // PLACE LEFT
    public static final double kLeftMotorPlaceCoralLeftTargetVelocity = 2800;
    public static final double kRightMotorPlaceCoralLeftTargetVelocity = -4300;
    // PLACE RIGHT
    public static final double kLeftMotorPlaceCoralRightTargetVelocity = 4300;
    public static final double kRightMotorPlaceCoralRightTargetVelocity = -2800;

    public static final int kCoralDetectedDistance = 60; //Was 50
    public static final int kCoralDetectedCountThreshold = 40;
    public static final int kPlaceCoralCommandRuntimeInMs = 1500;

    public static final int kReeflDetectedDistance = 380; // ~15.2 inches

    public static final boolean kLeftTargetVelocityFromDashboard = false;
    public static final boolean kRightTargetVelocityFromDashboard = false;
  }
  public enum EndEffectorState {
    UNKNOWN,
    CORAL_FREE,
    CORAL_LOADED;
  }
  public enum PlaceCoralDirection {
    PLACE_CORAL_STRAIGHT,
    PLACE_CORAL_RIGHT,
    PLACE_CORAL_LEFT;
  }
  // ---------------------------------------------

  // -------------------- ELEVATOR ---------------------
  public static final class ElevatorConstants {
    public static final int kLeftElevatorCanId = 3;
    public static final int kRightElevatorCanId = 4;
    public static final double kMaxOutRange = 0.4;  // FIXME: Was 0.2, can we speed elevator even more?
    public static final double kMinOutRange = -0.4;

    public static final boolean kTargetPositionFromDashboard = false;
  }
  // From REEFSCAPE Game Manual
  //   L1 = Trough, 1 ft. 6 in. (~46 cm) off the carpet
  //   L2 = 2 ft. 7-7/8 in. (~81 cm) from the carpet
  //   L3 = 3 ft. 11-5/8 in. (~121 cm) from the carpet
  //   L4 = 6 ft. (~183 cm) from the carpet
  public enum ElevatorState{
    UNKNOWN,
    P1(1.0),
    P1p5(6),   // This is level that allows AlgaeArm to go down
    P2(10),    // Was 11
    P3(28.0),
    P3p5(36),  // FIXME: Need to test, this is level that allows grab of higher reef algae
    P4(55.0);  // was 57

    private double elevatorPosition;
    ElevatorState(double elevatorPosition) {
      this.elevatorPosition = elevatorPosition;
    }
    ElevatorState() {}
    public double getPosition() {
      return elevatorPosition;
    }
  }
  // ---------------------------------------------

  // -------------------- ALGAE --------------------
  public static final class AlgaeConstants {
    public static final int kArmAlgaeMotorCanID = 9;
    public static final int kWheelAlgaeMotorCanID = 10;
    public static final double kAlgaeArmMaxOutRange = 0.4;
    public static final double kAlgaeArmMinOutRange = -0.4;

    public static final double kAlgaeWheelMaxOutRange = 0.8;
    public static final double kAlgaeWheelMinOutRange = -0.8;

    public static final double kAlgaeWheelAtReefTargetVelocity = 500;
    public static final double kAlgaeWheelAtReefHoldingTargetVelocity = 50;
    public static final double kAlgaeWheelAtProcessorReefAlgaeTargetVelocity = -1200;
    public static final double kAlgaeWheelAtProcessorGroundAlgaeTargetVelocity = 500;
    public static final double kAlgaeWheelAtGroundTargetVelocity = -300;

    public static final int kAlgaeWheelAtProcessorCommandRuntimeInMs = 1500;

    public static final boolean kArmTargetPositionFromDashboard = false;
    public static final boolean kWheelTargetVelocityFromDashboard = false;
  }
  public enum AlgaeArmState {
    UNKNOWN,
    ARM_STOWED(0),
    ARM_DOWN(0), // was 5, 3
    ARM_REEF_ALGAE_HOLD(15), // Was 20, FIXME: Need to test if works with robot touching reef
    ARM_REEF_ALGAE_RELEASE(18),// was 14
    ARM_GROUND_ALGAE_HOLD(35),
    ARM_GROUND_ALGAE_CATCH(33),
    ARM_GROUND_ALGAE_RELEASE(40);

    private double algaeArmPosition;
    AlgaeArmState(double algaeArmPosition) {
      this.algaeArmPosition = algaeArmPosition;
    }
    AlgaeArmState() {}
    public double getPosition() {
      return algaeArmPosition;
    }
  }
  // ---------------------------------------------

  // -------------------- CLIMBER --------------------
  public final class ClimberConstants {
    public static final int kClimberCanID = 19;
    public static final int kClimberPositionUp = 68; // Ideally 68
    public static final int kClimberPositionDown = -2; // FIXME: Try 0 once we have ratching gear box

    public static final double kmaxOutRange = 0.5;
    public static final double kminOutRange = -0.5;

    public static final double kServoAngleToEnableRatchet = 0.0;   // FIXME: Need to test to determine correct value.
    public static final double kServoAngleToDisableRatchet = 0.0;  // FIXME: Need to test to determine correct value.

    public static final boolean kTargetPositionFromDashboard = false;
    public static final boolean kServoAngleFromDashboard = true;
  }
  public enum ClimberState {
    UNKNOWN,
    CLIMBER_UP,
    CLIMBER_DOWN
  }
  // ---------------------------------------------

  // -------------------- LED --------------------
  public static final class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final double SOLID_DARK_GREEN = 0.75;
    public static final double SOLID_GREEN = 0.77;
    public static final double SOLID_SKY_BLUE = 0.83;
    public static final double SOLID_BLUE = 0.85;
    public static final double SOLID_DARK_BLUE = 0.87;
    public static final double SOLID_BLUE_VIOLET = 0.89;
    public static final double FIXED_PALETTE_PATTERN_FIRE_MEDIUM = -0.59;
    public static final double FIXED_PALETTE_PATTERN_FIRE_LARGE = -0.57;
  }
  public enum LEDColorState {
    NOTE_LESS,
    NOTE_DETECTED,
    SHOOTING;
  }
  // ---------------------------------------------

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.0;  // Orig 4.8, 2024Crecendo it was 4.0, 2025Reefscape trying slower
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionalSlewRate = 4; // radians per second; was .6
    public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%); was .9
    public static final double kRotationalSlewRate = 3; // percent per second (1 = 100%); was .9

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5); 
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 17;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 18;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = true;

    public static final double kSwerveSlideSpeed = 0.15;
    public static final double kSwerveBackupSpeed = 0.25;
    public static final int kSwerveBackupCommandRuntimeInMs = 700;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // was 0.0762 for 2024 Crescendo
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double kDriveDeadband = 0.065;  // was 0.05, 2024Crescendo used 0.065

  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

}

