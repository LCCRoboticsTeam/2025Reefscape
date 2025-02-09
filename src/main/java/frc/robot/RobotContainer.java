// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorState;
import frc.robot.Constants.OIConstants;

// Subsystems - imports
import frc.robot.subsystems.DriveSubsystem;
////import frc.robot.subsystems.ElevatorSubsystem;
////import frc.robot.subsystems.AlgaeSubsystem;
////import frc.robot.subsystems.ClimberSubsystem;
////import frc.robot.subsystems.EndEffectorSubsystem;
////import frc.robot.subsystems.LEDController;

// Commands - imports
//   Uses DriveSubsystem
import frc.robot.commands.SwerveGamepadDriveCommand;
import frc.robot.commands.SwerveSlideCommand;
//   Uses ElevatorSubsystem
//import frc.robot.commands.ChangeElevatorLevelCommand;
//   Uses EndEffector
////import frc.robot.commands.IntakeCommand;
//import frc.robot.commands.PlaceCoralCommand;
//   Uses AlgaeSubsystem
//import frc.robot.commands.RemoveReefAlgaeCommand;
//import frc.robot.commands.ProcessAlgaeCommand;
//import frc.robot.commands.PickupAlgaeCommand;
//   Uses ClimberSubsystem
//import frc.robot.commands.MoveClimberUpCommand;
//import frc.robot.commands.MoveClimberDownCommand;

// Cameras and Vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  //private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  //private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  //private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //private final LEDController ledController = new LEDController();

  // The driver's controllers
  private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final XboxController manipulatorXboxController = new XboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  // Dashboard - Choosers
  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Cameras and Vision
  UsbCamera reefsideUsbCamera = CameraServer.startAutomaticCapture(0);
  //UsbCamera climbersideUsbCamera = CameraServer.startAutomaticCapture(1);
  //PhotonCamera frontsidePhotonCamera = new PhotonCamera("Frontside");
  //PhotonCamera backsidePhotonCamera = new PhotonCamera("Backside");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     // Register Named Commands
     ////NamedCommands.registerCommand("IntakeCoral", new IntakeCommand(endEffectorSubsystem));
     ////NamedCommands.registerCommand("PlaceCoral", new PlaceCommand(endEffectorSubsystem));
     NamedCommands.registerCommand("SwerveSlideRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed));
     NamedCommands.registerCommand("SwerveSlideLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed));


     // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative", true);
    fieldRelativeChooser.addOption("Robot Relative", false);
    SmartDashboard.putData(fieldRelativeChooser);
    //SmartDashboard.putData(autoChooser);
     
    // Commands launched from Dashboard
    //SmartDashboard.putData("IntakeCoral", NamedCommands.getCommand("IntakeCoral"));
    //SmartDashboard.putData("PlaceCoral", NamedCommands.getCommand("PlaceCoral"));

    // Configure default commands
    driveSubsystem.setDefaultCommand(new SwerveGamepadDriveCommand(driveSubsystem, driverCommandXboxController::getLeftX,
      driverCommandXboxController::getLeftY, driverCommandXboxController::getRightX, fieldRelativeChooser::getSelected));

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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driverCommandXboxController.rightBumper().whileTrue(NamedCommands.getCommand("SwerveSlideRight"));
    driverCommandXboxController.leftBumper().whileTrue(NamedCommands.getCommand("SwerveSlideLeft"));

    //manipulatorCommandXboxController.a().and(new Trigger(endEffectorSubsystem::isCoralNotLoaded)).onTrue(NamedCommands.getCommand("IntakeCoral"));
    //manipulatorCommandXboxController.a().and(new Trigger(endEffectorSubsystem::isCoralLoaded)).onTrue(NamedCommands.getCommand("PlaceCoralCommand"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
