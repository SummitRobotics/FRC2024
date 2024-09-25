// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbDefault;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.ShooterAutomation;
import frc.robot.commands.SuperstructureDefault;
import frc.robot.commands.SwerveArcade;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.Controller;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.HyperionDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveBotDrivetrain;
import frc.robot.devices.LEDs.LEDs;
import frc.robot.devices.LEDs.LEDCall;
import frc.robot.devices.LEDs.LEDCalls;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /** The robot we are deploying to. */
  private enum Hardware {
    HYPERION,
    SWERVEBOT
  }

  private final Hardware hardware = Hardware.HYPERION;

  // The robot's subsystems and commands are defined here..
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AHRS gyro = new AHRS();
  private Swerve drivetrain;
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private Superstructure superstructure;
  private Intake intake;
  private Climb climb;
  private PowerDistribution pdp;

  // Instantiate USB devices
  private final Controller driverController
      = new Controller(OperatorConstants.kDriverControllerPort);
  private Controller gunnerController
      = new Controller(OperatorConstants.kGunnerControllerPort);
  private ButtonBox buttonBox
      = new ButtonBox(OperatorConstants.kButtonBoxPort);

  private SwerveArcade drivetrainDefault;
  private SuperstructureDefault superstructureDefault;
  private IntakeDefault intakeDefault;
  private ClimbDefault climbDefault;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate
    switch (hardware) {
      case SWERVEBOT:
        drivetrain = new SwerveBotDrivetrain(gyro);
        break;
      case HYPERION:
        drivetrain = new HyperionDrivetrain(gyro);
        intake = new Intake();
        intakeDefault = new IntakeDefault(
            intake,
            superstructure,
            new Trigger(() -> gunnerController.getYButton()), // manualOverrideSupplier
            () -> -gunnerController.getLeftY(), // manualPivot
            // () -> gunnerController.getLeftY(),
            () -> gunnerController.getLeftX() // manualRoller
        );
        superstructure = new Superstructure();
        superstructureDefault = new SuperstructureDefault(
            superstructure,
            intake,
            buttonBox,
            buttonBox.getReceivePreset(), // receiveSupplier
            buttonBox.getAmpPreset(), // ampSupplier
            new Trigger(() -> false), // trapSupplier
            buttonBox.getSpeakerPreset(), // shootSupplier
            buttonBox.getPodiumPreset(), // Podium
            () -> (gunnerController.getXButton() ? 1 : 0) - (false ? 1 : 0), // elevatorManualSupplier
            () -> gunnerController.getAButton() ? 1 : 0, // shooterManualSupplier
            () -> gunnerController.getBButton() ? 0.8 : 0, // indexerManualSupplier
            () -> gunnerController.getRightX(), // pivotManualSupplier
            buttonBox.getShoot(), // shootConfirm
            new Trigger(() -> buttonBox.getRawButton(8)), // Spit
            new Trigger(() -> buttonBox.getRawButton(7)), // Spit far
            new Trigger(() -> buttonBox.getRawButton(9)) // Baby bird source pickup
        );
        climb = new Climb();

        climbDefault = new ClimbDefault(
            climb,
            intake,
            gyro,
            new Trigger(() -> false),
            new Trigger(() -> gunnerController.getLeftBumper()),
            new Trigger(() -> gunnerController.getRightBumper()),
            () ->  gunnerController.getLeftTrigger(),
            () -> gunnerController.getRightTrigger(),
            new Trigger(() -> false)
        );
        // SmartDashboard.putData("Intake", intake);
        SmartDashboard.putData("Elevator / Shooter", superstructure);
        // SmartDashboard.putData("Climb", climb);
        // SmartDashboard.putData(CommandScheduler.getInstance());
        // Intake recalibrate
        // new Trigger(() -> driverController.getRightBumper()).onTrue(new InstantCommand(() -> {
          // Intake.pivot.getEncoder().setPosition(IntakeState.DOWN.pivot);
          // intake.setState(IntakeState.DOWN);
        // }));
        new Trigger(() -> buttonBox.getRawButton(12)).onTrue(new InstantCommand(() -> {
          // LEDCalls.ON.cancel();
          LEDCalls.AMPLIFY_RED.cancel();
          LEDCalls.AMPLIFY_BLUE.activate();
        }));
        new Trigger(() -> buttonBox.getRawButton(14)).onTrue(new InstantCommand(() -> {
          // LEDCalls.ON.cancel();
          LEDCalls.AMPLIFY_BLUE.cancel();
          LEDCalls.AMPLIFY_RED.activate();
        }));
        new Trigger(() -> buttonBox.getRawButton(13)).onTrue(new InstantCommand(() -> {
          LEDCalls.AMPLIFY_BLUE.cancel();
          LEDCalls.AMPLIFY_RED.cancel();
          // LEDCalls.ON.activate();
        }));
        intake.setDefaultCommand(intakeDefault);
        superstructure.setDefaultCommand(superstructureDefault);
        climb.setDefaultCommand(climbDefault);

        pdp = new PowerDistribution();
        // SmartDashboard.putData("PDP", new Sendable() {
          // @Override
          // public void initSendable(SendableBuilder builder) {
            // builder.addDoubleProperty("Current Draw", pdp::getTotalCurrent, null);
          // }
        // });
        break;
      default:
        break;
    }

    drivetrainDefault = new SwerveArcade(
        drivetrain,
        superstructure,
        gyro,
        () -> driverController.getLeftY(), // fwd
        () -> driverController.getLeftX(), // str
        () -> driverController.getRightX(), // rcw
        new Trigger(() -> driverController.getBButton()), // resetPose
        new Trigger(() -> driverController.getAButton()), // flipMode
        new Trigger(() -> false) // lock rotation
    );

    // new Trigger(() -> buttonBox.getRawButton(4)).whileTrue(new ShooterAutomation(drivetrain, superstructure, intake, () -> driverController.getLeftY(), () -> driverController.getLeftX()));
    new Trigger(() -> buttonBox.getRawButton(4)).whileTrue(new ShooterAutomation(drivetrain, superstructure, intake, () -> 0, () -> 0));

    drivetrain.setDefaultCommand(drivetrainDefault);

    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand(() -> {}));
    // autoChooser.addOption("One Piece", Autos.onePiece(superstructure, intake));
    // autoChooser.addOption("Two Piece", Autos.twoPiece(drivetrain, superstructure, intake));
    // autoChooser.addOption("Two Piece Open Side", Autos.twoPieceOpenSide(drivetrain, superstructure, intake));
    // autoChooser.addOption("N Piece", Autos.nPiece(drivetrain, superstructure, intake));
    // autoChooser.addOption("Auto Shoot", new ShooterAutomation(drivetrain, superstructure, intake));
    // autoChooser.addOption("Amp Side", Autos.twoPieceAmpSide(drivetrain, superstructure, intake));
    // autoChooser.addOption("Shoot Test", Autos.splineShoot(drivetrain, superstructure, intake));
    // autoChooser.addOption("Far Outer", Autos.far(drivetrain, superstructure, intake, true));
    // autoChooser.addOption("Far Inner", Autos.far(drivetrain, superstructure, intake, false));
    // autoChooser.addOption("Center Under", Autos.center(drivetrain, superstructure, intake));
    // autoChooser.addOption("Wing", Autos.wing(drivetrain, superstructure, intake));
    SmartDashboard.putData("Drivetrain", drivetrain);
    SmartDashboard.putData("Auto Choice", autoChooser);
    // SmartDashboard.putData("Gyro", new Sendable() {
    // public void initSendable(SendableBuilder builder) {
    // builder.addFloatProperty("Pitch", gyro::getPitch, null);
    // builder.addFloatProperty("Yaw", gyro::getYaw, null);
    // builder.addFloatProperty("Roll", gyro::getRoll, null);
    // }
    // });

    // Flash all lights on the button box to indicate that the robot is ready.
    flashAllButtonBoxLights();
  }

  public void autonomousPeriodic() {
    // drivetrain.drive(new ChassisSpeeds(-1, 0, 0));
  }

  /*
   * Flash the lights on the button box.
   */
  private void flashAllButtonBoxLights() {
    buttonBox.AllLED(true);
    new java.util.Timer().schedule(
        new java.util.TimerTask() {
          @Override
          public void run() {
            buttonBox.AllLED(false);
          }
        },
        500);
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return true ? autoChooser.getSelected()
      : new ShooterAutomation(drivetrain, superstructure, intake);
  }

  /** Robot periodic method. */
  public void robotPeriodic() {
    // PPLibTelemetry.setCurrentPose(drivetrain.getPose());
    // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("testpath"));
    if (hardware == Hardware.HYPERION) {
      buttonBox.sendMessage();
    }
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    if (superstructure.getState() == SuperstructureState.RECEIVE) {
      LEDCalls.IDLE.cancel();
      LEDCalls.RECEIVING.activate();
      LEDCalls.MO.cancel();
    }
    
    if (intake.getState() == IntakeState.MANUAL_OVERRIDE) {
      LEDCalls.IDLE.cancel();
      LEDCalls.RECEIVING.cancel();
      LEDCalls.MO.activate();
    } else {
      LEDCalls.MO.cancel();
    }

    if (superstructure.getState() == SuperstructureState.IDLE) {
      LEDCalls.IDLE.activate();
      LEDCalls.MO.cancel();
    }
  }

  public void robotInit() {
    // DataLogManager.start();
    // URCL.start();
    LEDCalls.ON.activate();

  }

  public void disabledInit() {
    LEDs.getInstance().removeAllCalls();
    LEDCalls.ON.activate();
  }
}
