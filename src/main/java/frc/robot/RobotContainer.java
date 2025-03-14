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
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems defined here...
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  private final AlgaeArmSubsystem algaeArmSubsystem = new AlgaeArmSubsystem();
  private final AlgaeWheelSubsystem algaeWheelSubsystem = new AlgaeWheelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDController ledController;

  // The driver's controllers
  private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);

  //private final XboxController manipulatorXboxController = new XboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  //private final CommandLaunchpadController commandLaunchpad = new CommandLaunchpadController(OIConstants.kLaunchpadControllerPort);

  // Dashboard - Choosers
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> reefAlgaeChooser;

  // Cameras and Vision
  UsbCamera reefsideUsbCamera = CameraServer.startAutomaticCapture(1);
  UsbCamera climbersideUsbCamera = CameraServer.startAutomaticCapture(0);
  PhotonCamera frontsidePhotonCamera = new PhotonCamera("frontsidePhotonCam");
  PhotonCamera backsidePhotonCamera = new PhotonCamera("backsidePhotonCam");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(BooleanSupplier isRobotEnabled) {
    driveSubsystem = new DriveSubsystem(frontsidePhotonCamera, backsidePhotonCamera);
    ledController = new LEDController(isRobotEnabled, endEffectorSubsystem::isCoralLoaded, algaeWheelSubsystem::isAlgaeLoaded);

    // We always start at P1 level
    elevatorSubsystem.setElevatorState(ElevatorState.P1);
    // We always start at ARM_STOWED
    algaeArmSubsystem.setAlgaeArmState(AlgaeArmState.ARM_STOWED);
    // We always start at ALGAE_FREE 
    algaeWheelSubsystem.setAlgaeWheelState(AlgaeWheelState.ALGAE_FREE);
    // We always start with CLIMBER_DOWN and the Ratchet disabled
    climberSubsystem.setClimberState(ClimberState.CLIMBER_DOWN);
    climberSubsystem.setServoAngle(ClimberConstants.kServoAngleToDisableRatchet);

    // Register Named Commands
    NamedCommands.registerCommand("IntakeCoral", new IntakeCommand(endEffectorSubsystem));
    NamedCommands.registerCommand("PlaceCoralStraight", new SequentialCommandGroup(new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_STRAIGHT, elevatorSubsystem::isElevatorAtP4),
                                                                                        new ConditionalCommand(new ElevatorDownCommand(elevatorSubsystem, true), 
                                                                                                               new ParallelCommandGroup(new SwerveBackupCommand(driveSubsystem, DriveConstants.kSwerveBackupSpeed),
                                                                                                                                        new ElevatorDownCommand(elevatorSubsystem, true)),
                                                                                                               elevatorSubsystem::isElevatorAtP1)));
    NamedCommands.registerCommand("PlaceCoralRight", new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_RIGHT, elevatorSubsystem::isElevatorAtP4));
    NamedCommands.registerCommand("PlaceCoralLeft", new PlaceCoralCommand(endEffectorSubsystem, PlaceCoralDirection.PLACE_CORAL_LEFT, elevatorSubsystem::isElevatorAtP4));
    NamedCommands.registerCommand("ElevatorUp", new ElevatorUpCommand(elevatorSubsystem, false, endEffectorSubsystem::isCoralLoaded));
    NamedCommands.registerCommand("ElevatorDown", new ElevatorDownCommand(elevatorSubsystem, false));
    NamedCommands.registerCommand("ClimberUp", new MoveClimberUpCommand(climberSubsystem));
    NamedCommands.registerCommand("ClimberDown", new MoveClimberDownCommand(climberSubsystem));
    NamedCommands.registerCommand("SwerveSlideRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed, false, endEffectorSubsystem::getReefsideDistanceMM));
    NamedCommands.registerCommand("SwerveSlideLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed, false, endEffectorSubsystem::getReefsideDistanceMM));
    NamedCommands.registerCommand("AutoReefAlignmentRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveAutoAlignSlideSpeed, true, endEffectorSubsystem::getReefsideDistanceMM));
    NamedCommands.registerCommand("AutoReefAlignmentLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveAutoAlignSlideSpeed, true, endEffectorSubsystem::getReefsideDistanceMM));
    NamedCommands.registerCommand("SwerveToCenterReef", new SwerveToCenterReefCommand(driveSubsystem, frontsidePhotonCamera));
    NamedCommands.registerCommand("SwerveToCenterCoralStation", new SwerveToCenterCoralStationCommand(driveSubsystem, backsidePhotonCamera));
    NamedCommands.registerCommand("GrabAlgaeFromReef", new SequentialCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN), 
                                                                                       new AlgaeWheelAtReefCommand(algaeWheelSubsystem, true, false),
                                                                                       new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_HOLD), 
                                                                                       new AlgaeWheelAtReefCommand(algaeWheelSubsystem, false, false),
                                                                                       new ConditionalCommand(new SequentialCommandGroup(new SwerveBackupCommand(driveSubsystem, DriveConstants.kSwerveBackupSpeed),
                                                                                                                                         new ElevatorDownCommand(elevatorSubsystem, true)), 
                                                                                                              new SequentialCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN), 
                                                                                                                                         new ElevatorDownCommand(elevatorSubsystem, true)),
                                                                                                              algaeWheelSubsystem::isAlgaeLoadedFromReef)));
    NamedCommands.registerCommand("ProcessAlgaeFromReef", new SequentialCommandGroup(new ParallelCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_RELEASE), 
                                                                                                                   new AlgaeWheelAtProcessorCommand(algaeWheelSubsystem, true)), 
                                                                                          new ElevatorUpCommand(elevatorSubsystem, true, endEffectorSubsystem::isCoralLoaded),
                                                                                          new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                          new ElevatorDownCommand(elevatorSubsystem, true)));
    NamedCommands.registerCommand("GrabAlgaeFromGround", new SequentialCommandGroup(new ElevatorUpCommand(elevatorSubsystem, true, endEffectorSubsystem::isCoralLoaded),
                                                                                         new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_GROUND_ALGAE_CATCH),
                                                                                         new ElevatorDownCommand(elevatorSubsystem, true),
                                                                                         new AlgaeWheelAtGroundCommand(algaeWheelSubsystem, false),
                                                                                         new ConditionalCommand(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_GROUND_ALGAE_HOLD), 
                                                                                                                new SequentialCommandGroup(new ElevatorUpCommand(elevatorSubsystem, true, endEffectorSubsystem::isCoralLoaded),
                                                                                                                                           new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                                                                           new ElevatorDownCommand(elevatorSubsystem, true)),
                                                                                                                algaeWheelSubsystem::isAlgaeLoadedFromGround)));
    NamedCommands.registerCommand("ProcessAlgaeFromGround", new SequentialCommandGroup(algaeArmSubsystem.setArmMotorIdleModeToCoastCommand(),
                                                                                            new ParallelCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_GROUND_ALGAE_RELEASE), 
                                                                                                                     new AlgaeWheelAtProcessorCommand(algaeWheelSubsystem, false)), 
                                                                                            algaeArmSubsystem.setArmMotorIdleModeToBrakeCommand(),
                                                                                            new ElevatorUpCommand(elevatorSubsystem, true, endEffectorSubsystem::isCoralLoaded),
                                                                                            new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                            new ElevatorDownCommand(elevatorSubsystem, true)));
    NamedCommands.registerCommand("TossRightAlgaeFromReef", new SequentialCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN), 
                                                                                            new AlgaeWheelAtReefCommand(algaeWheelSubsystem, true, false),
                                                                                            new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_HOLD), 
                                                                                            new AlgaeWheelAtReefCommand(algaeWheelSubsystem, false, true),
                                                                                            new SwerveBackupCommand(driveSubsystem, DriveConstants.kSwerveBackupSpeed),
                                                                                            new ParallelCommandGroup(new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateRightSpeed),
                                                                                                                     new ElevatorDownCommand(elevatorSubsystem, true)),
                                                                                            NamedCommands.getCommand("ProcessAlgaeFromReef")));
                                                                                            //new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateLeftSpeed))); 
    NamedCommands.registerCommand("InitAlgaeSystem", new SequentialCommandGroup(new AlgaeWheelCommand(algaeWheelSubsystem, true), 
                                                                                            new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
                                                                                            new ElevatorDownCommand(elevatorSubsystem, true)));                                                                                            
    NamedCommands.registerCommand("TossLeftAlgaeFromReef", new SequentialCommandGroup(new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN), 
                                                                                           new AlgaeWheelAtReefCommand(algaeWheelSubsystem, true, false),
                                                                                           new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_HOLD), 
                                                                                           new AlgaeWheelAtReefCommand(algaeWheelSubsystem, false, true),
                                                                                           new SwerveBackupCommand(driveSubsystem, DriveConstants.kSwerveBackupSpeed),
                                                                                           new ParallelCommandGroup(new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateLeftSpeed),
                                                                                                                    new ElevatorDownCommand(elevatorSubsystem, true)),
                                                                                           NamedCommands.getCommand("ProcessAlgaeFromReef")));
                                                                                           //new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateRightSpeed))); 
   // NamedCommands.registerCommand("LaunchAlgaeIntoBarge", new SequentialCommandGroup(new ElevatorUpCommand(elevatorSubsystem, false, endEffectorSubsystem::isCoralLoaded),
   //                                                                                       new ElevatorUpCommand(elevatorSubsystem, false, endEffectorSubsystem::isCoralLoaded),
   //                                                                                       new WaitCommand(0.5),
   //                                                                                       new AlgaeWheelCommand(algaeWheelSubsystem, false), 
   //                                                                                       new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_REEF_ALGAE_LAUNCH),
   //                                                                                       new AlgaeWheelCommand(algaeWheelSubsystem, true),
   //                                                                                       new AlgaeArmCommand(algaeArmSubsystem, AlgaeArmState.ARM_DOWN),
   //                                                                                       new ElevatorDownCommand(elevatorSubsystem, true)));
    NamedCommands.registerCommand("PlaceCoralOnLeftL2", new SequentialCommandGroup(NamedCommands.getCommand("ElevatorUp"),
                                                                                        NamedCommands.getCommand("AutoReefAlignmentLeft"),
                                                                                        NamedCommands.getCommand("PlaceCoralStraight")));
    NamedCommands.registerCommand("PlaceCoralOnRightL2", new SequentialCommandGroup(NamedCommands.getCommand("ElevatorUp"),
                                                                                         NamedCommands.getCommand("AutoReefAlignmentRight"),
                                                                                         NamedCommands.getCommand("PlaceCoralStraight")));
    NamedCommands.registerCommand("PlaceCoralOnLeftL3", new SequentialCommandGroup(NamedCommands.getCommand("ElevatorUp"), 
                                                                                        NamedCommands.getCommand("ElevatorUp"),
                                                                                        NamedCommands.getCommand("AutoReefAlignmentLeft"),
                                                                                        NamedCommands.getCommand("PlaceCoralStraight")));
    NamedCommands.registerCommand("PlaceCoralOnRightL3", new SequentialCommandGroup(NamedCommands.getCommand("ElevatorUp"), 
                                                                                         NamedCommands.getCommand("ElevatorUp"),
                                                                                         NamedCommands.getCommand("AutoReefAlignmentRight"),
                                                                                         NamedCommands.getCommand("PlaceCoralStraight")));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("LeftReef3Coral");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    reefAlgaeChooser = new SendableChooser<>();
    reefAlgaeChooser.setDefaultOption("TossRightAlgaeFromReef", NamedCommands.getCommand("TossRightAlgaeFromReef"));
    reefAlgaeChooser.addOption("TossLeftAlgaeFromReef", NamedCommands.getCommand("TossLeftAlgaeFromReef"));
    reefAlgaeChooser.addOption("GrabAlgaeFromReef", NamedCommands.getCommand("GrabAlgaeFromReef"));
    SmartDashboard.putData("Reef Algae Chooser", reefAlgaeChooser);

    // Configure the trigger bindings
    configureBindings();
     
    // Commands launched from Dashboard (Example format below)
    SmartDashboard.putData("Reset Gyro Heading", driveSubsystem.zeroHeadingCommand());
    SmartDashboard.putData("Reverse Intake", new ReverseIntakeCommand(endEffectorSubsystem));
    SmartDashboard.putData("Reset Algae Subsystem", NamedCommands.getCommand("InitAlgaeSystem"));
    SmartDashboard.putData("Reset Elevator Position", elevatorSubsystem.resetPositionCommand());

    //SmartDashboard.putData("GrabAlgaeFromReef", NamedCommands.getCommand("GrabAlgaeFromReef"));
    //SmartDashboard.putData("LaunchAlgaeIntoBarge", NamedCommands.getCommand("LaunchAlgaeIntoBarge"));
    //SmartDashboard.putData("Rotate Right", new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateRightSpeed));
    //SmartDashboard.putData("Rotate Left", new SwerveRotateCommand(driveSubsystem, DriveConstants.kSwerveRotateLeftSpeed));

    // Configure default commands
    driveSubsystem.setDefaultCommand(new SwerveGamepadDriveCommand(driveSubsystem, driverCommandXboxController::getLeftX,
      driverCommandXboxController::getLeftY, driverCommandXboxController::getRightX, climberSubsystem::isClimberUp, 
      driverXboxController::getLeftStickButton, frontsidePhotonCamera));

    // Camera settings
    //reefsideUsbCamera.setResolution(640, 480);
    //climbersideUsbCamera.setResolution(320, 280);

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
    driverCommandXboxController.rightBumper().and(new Trigger(algaeArmSubsystem::isArmStowedOrDown)).
    whileTrue(NamedCommands.getCommand("SwerveSlideRight"));
    driverCommandXboxController.leftBumper().and(new Trigger(algaeArmSubsystem::isArmStowedOrDown)).
    whileTrue(NamedCommands.getCommand("SwerveSlideLeft"));
    driverCommandXboxController.b().whileTrue(NamedCommands.getCommand("AutoReefAlignmentRight"));
    driverCommandXboxController.x().whileTrue(NamedCommands.getCommand("AutoReefAlignmentLeft"));
    driverCommandXboxController.y().onTrue(NamedCommands.getCommand("ElevatorUp"));
    driverCommandXboxController.a().and(new Trigger(elevatorSubsystem::isElevatorNotAtP1)).onTrue(NamedCommands.getCommand("ElevatorDown"));
    driverCommandXboxController.rightTrigger().whileTrue(NamedCommands.getCommand("SwerveToCenterReef"));
    driverCommandXboxController.leftTrigger().whileTrue(NamedCommands.getCommand("SwerveToCenterCoralStation"));

    // MANIPULATOR XBOX Controller
    manipulatorCommandXboxController.a().onTrue(NamedCommands.getCommand("IntakeCoral"));
    manipulatorCommandXboxController.x().onTrue(NamedCommands.getCommand("PlaceCoralLeft"));
    manipulatorCommandXboxController.y().onTrue(NamedCommands.getCommand("PlaceCoralStraight"));
    manipulatorCommandXboxController.b().onTrue(NamedCommands.getCommand("PlaceCoralRight"));
    manipulatorCommandXboxController.back().onTrue(NamedCommands.getCommand("ClimberUp"));
    manipulatorCommandXboxController.start().onTrue(NamedCommands.getCommand("ClimberDown"));
    manipulatorCommandXboxController.leftBumper().and(new Trigger(elevatorSubsystem::isElevatorNotAtP1orP1p5)).
                                                  onTrue(reefAlgaeChooser.getSelected());
    manipulatorCommandXboxController.leftBumper().and(new Trigger(elevatorSubsystem::isElevatorAtP1orP1p5)).
                                                  onTrue(NamedCommands.getCommand("GrabAlgaeFromGround"));
    manipulatorCommandXboxController.rightBumper().and(new Trigger(elevatorSubsystem::isElevatorAtP1)).
                                                   and(new Trigger(algaeWheelSubsystem::isAlgaeLoadedFromReef)).
                                                   onTrue(NamedCommands.getCommand("ProcessAlgaeFromReef"));
    manipulatorCommandXboxController.rightBumper().and(new Trigger(elevatorSubsystem::isElevatorAtP1)).
                                                   and(new Trigger(algaeWheelSubsystem::isAlgaeLoadedFromGround)).
                                                   onTrue(NamedCommands.getCommand("ProcessAlgaeFromGround"));

    // Custom LAUNCHPAD Controller
    //commandLaunchpad.l2Left().onTrue(NamedCommands.getCommand("PlaceCoralOnLeftL2"));
    //commandLaunchpad.l2Right().onTrue(NamedCommands.getCommand("PlaceCoralOnRightL2"));
    //commandLaunchpad.l2Left().onTrue(NamedCommands.getCommand("PlaceCoralOnLeftL3"));
    //commandLaunchpad.l2Right().onTrue(NamedCommands.getCommand("PlaceCoralOnRightL3"));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Start with a CORAL in the EndEffector to start
    endEffectorSubsystem.setEndEffectorState(EndEffectorState.CORAL_LOADED);
    // All autos start with robot facing the Drive Station
    driveSubsystem.robotFacingDriveStation();

    return autoChooser.getSelected();
    //return null;
  }

  public void zeroHeading() {
    driveSubsystem.zeroHeading();
  }
}
