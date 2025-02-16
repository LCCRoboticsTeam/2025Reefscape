// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// Cameras and Vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.math.controller.ElevatorFeedforward;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  private final AlgaeArmSubsystem algaeArmSubsystem = new AlgaeArmSubsystem();
  private final AlgaeWheelSubsystem algaeWheelSubsystem = new AlgaeWheelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //private final LEDController ledController = new LEDController();

  // The driver's controllers
  //private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);

  //private final XboxController manipulatorXboxController = new XboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  // Dashboard - Choosers
  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Cameras and Vision
  UsbCamera reefsideUsbCamera = CameraServer.startAutomaticCapture(1);
  UsbCamera climbersideUsbCamera = CameraServer.startAutomaticCapture(0);
  PhotonCamera frontsidePhotonCamera = new PhotonCamera("Frontside");
  PhotonCamera backsidePhotonCamera = new PhotonCamera("Backside");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // We always start at P1 level
    elevatorSubsystem.setElevatorState(ElevatorState.P1);
    // We always start at ARM_DOWN
    algaeArmSubsystem.setAlgaeArmState(AlgaeArmState.ARM_STOWED);
    // We always start with CLIMBER_DOWN
    climberSubsystem.setClimberState(ClimberState.CLIMBER_DOWN);

     // Register Named Commands
     NamedCommands.registerCommand("IntakeCoral", new IntakeCommand(endEffectorSubsystem));
     NamedCommands.registerCommand("PlaceCoralStraight", new SequentialCommandGroup(new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_STRAIGHT, elevatorSubsystem::isElevatorAtP4),
                                                                                         new ElevatorDownCommand(elevatorSubsystem, true)));
                                                                                         //new ElevatorDownCommand(elevatorSubsystem, true),
                                                                                         //new SwerveBackupCommand(driveSubsystem, DriveConstants.kSwerveBackupSpeed)));
     NamedCommands.registerCommand("PlaceCoralRight", new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_RIGHT, elevatorSubsystem::isElevatorAtP4));
     NamedCommands.registerCommand("PlaceCoralLeft", new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_LEFT, elevatorSubsystem::isElevatorAtP4));
     NamedCommands.registerCommand("ElevatorUp", new ElevatorUpCommand(elevatorSubsystem, false));
     NamedCommands.registerCommand("ElevatorDown", new ElevatorDownCommand(elevatorSubsystem, false));
     NamedCommands.registerCommand("ClimberUp", new MoveClimberUpCommand(climberSubsystem));
     NamedCommands.registerCommand("ClimberDown", new MoveClimberDownCommand(climberSubsystem));
     NamedCommands.registerCommand("SwerveSlideRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed, false, endEffectorSubsystem::getReefsideDistanceMM));
     NamedCommands.registerCommand("SwerveSlideLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed, false, endEffectorSubsystem::getReefsideDistanceMM));
     NamedCommands.registerCommand("AutoReefAlignmentRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed, true, endEffectorSubsystem::getReefsideDistanceMM));
     NamedCommands.registerCommand("AutoReefAlignmentLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed, true, endEffectorSubsystem::getReefsideDistanceMM));
     NamedCommands.registerCommand("GrabAlgaeFromReef", new SequentialCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                        new ParallelRaceGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_HOLD), 
                                                                                                              new AlgaeWheelAtReefCommand(algaeWheelSubsystem))));
     NamedCommands.registerCommand("ProcessAlgaeFromReef", new SequentialCommandGroup(new ParallelCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_RELEASE), 
                                                                                                                    new AlgaeWheelAtProcessorCommand(algaeWheelSubsystem, true)), 
                                                                                           new ElevatorUpCommand(elevatorSubsystem, true),
                                                                                           new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                           new ElevatorDownCommand(elevatorSubsystem, true)));
     // GrabAlgaeFromGround - This will be a command sequence
     //NamedCommands.registerCommand("ProcessAlgaeFromGround", new SequentialCommandGroup(new ParallelCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_GROUND_ALGAE_RELEASE), 
     //                                                                                                            new AlgaeWheelAtProcessorCommand(algaeWheelSubsystem, false)), 
     //                                                                                        new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN)));

     // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative", true);
    fieldRelativeChooser.addOption("Robot Relative", false);
    SmartDashboard.putData(fieldRelativeChooser);
    SmartDashboard.putData(autoChooser);
     
    // Commands launched from Dashboard (Example format below)
    //SmartDashboard.putData("IntakeCoral", NamedCommands.getCommand("IntakeCoral"));

    // Configure default commands
    driveSubsystem.setDefaultCommand(new SwerveGamepadDriveCommand(driveSubsystem, driverCommandXboxController::getLeftX,
      driverCommandXboxController::getLeftY, driverCommandXboxController::getRightX, fieldRelativeChooser::getSelected, climberSubsystem::isClimberUp));

    // Camera settings
    //reefsideUsbCamera.setResolution(640, 480);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // XBOX Controller Diagram
    //   https://gist.github.com/palmerj/586375bcc5bc83ccdaf00c6f5f863e86

    // DRIVER XBOX Controller
    //   Note: Right stick and Left stick already mapped via SwerveGamepadDriveCommand() in earlier code
    driverCommandXboxController.rightBumper().whileTrue(NamedCommands.getCommand("SwerveSlideRight"));
    driverCommandXboxController.leftBumper().whileTrue(NamedCommands.getCommand("SwerveSlideLeft"));
    driverCommandXboxController.b().whileTrue(NamedCommands.getCommand("AutoReefAlignmentRight"));
    driverCommandXboxController.x().whileTrue(NamedCommands.getCommand("AutoReefAlignmentLeft"));
    driverCommandXboxController.y().onTrue(NamedCommands.getCommand("ElevatorUp"));
    driverCommandXboxController.a().onTrue(NamedCommands.getCommand("ElevatorDown"));

    // MANIPULATOR XBOX Controller
    //manipulatorCommandXboxController.a().and(new Trigger(endEffectorSubsystem::isCoralNotLoaded)).onTrue(NamedCommands.getCommand("IntakeCoral"));
    manipulatorCommandXboxController.a().onTrue(NamedCommands.getCommand("IntakeCoral"));
    //manipulatorCommandXboxController.a().and(new Trigger(endEffectorSubsystem::isCoralLoaded)).onTrue(NamedCommands.getCommand("PlaceCoral"));
    manipulatorCommandXboxController.x().onTrue(NamedCommands.getCommand("PlaceCoralLeft"));
    manipulatorCommandXboxController.y().onTrue(NamedCommands.getCommand("PlaceCoralStraight"));
    manipulatorCommandXboxController.b().onTrue(NamedCommands.getCommand("PlaceCoralRight"));
    manipulatorCommandXboxController.back().onTrue(NamedCommands.getCommand("ClimberUp"));
    manipulatorCommandXboxController.start().onTrue(NamedCommands.getCommand("ClimberDown"));
    manipulatorCommandXboxController.leftBumper().and(new Trigger(elevatorSubsystem::isElevatorNotAtP1)).whileTrue(NamedCommands.getCommand("GrabAlgaeFromReef"));
    //manipulatorCommandXboxController.rightBumper().and(new Trigger(elevatorSubsystem::isElevatorAtP1)).onTrue(NamedCommands.getCommand("ProcessAlgaeFromReef"));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Start with a CORAL in the EndEffector to start
    endEffectorSubsystem.setEndEffectorState(EndEffectorState.CORAL_LOADED);
    return autoChooser.getSelected();
    //return null;
  }
}
