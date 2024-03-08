// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.commands.FollowPathPlannerTrajectory;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.SuperstructureDefault;
import frc.robot.commands.SwerveArcade;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.Controller;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.HyperionDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveBotDrivetrain;
import frc.robot.devices.LEDs.LEDs;
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
            new Trigger(() -> driverController.getXButton()), // pivotUpandDown
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
            buttonBox.getTrapPreset(), // trapSupplier
            buttonBox.getSpeakerPreset(), // shootSupplier
            buttonBox.getPodiumPreset(), // Podium
            () -> (gunnerController.getXButton() ? 1 : 0) - (gunnerController.getBButton() ? 1 : 0), // elevatorManualSupplier
            () -> gunnerController.getAButton() ? 1 : 0, // shooterManualSupplier
            () -> gunnerController.getRightY(), // indexerManualSupplier
            () -> gunnerController.getRightX(), // pivotManualSupplier
            buttonBox.getShoot(),
            new Trigger(() -> buttonBox.getRawButton(8)) // shootConfirm
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
            new Trigger(() -> buttonBox.getRawButton(9))
        );
        // SmartDashboard.putData("Intake", intake);
        // SmartDashboard.putData("Elevator / Shooter", superstructure);
        // SmartDashboard.putData("Climb", climb);
        // SmartDashboard.putData(CommandScheduler.getInstance());
        // Intake recalibrate
        new Trigger(() -> driverController.getRightBumper()).onTrue(new InstantCommand(() -> {
          Intake.pivot.getEncoder().setPosition(IntakeState.DOWN.pivot);
          intake.setState(IntakeState.DOWN);
        }));
        new Trigger(() -> buttonBox.getRawButton(12)).onTrue(new InstantCommand(() -> {
          LEDCalls.ON.cancel();
          LEDCalls.AMPLIFY_RED.cancel();
          LEDCalls.AMPLIFY_BLUE.activate();
        }));
        new Trigger(() -> buttonBox.getRawButton(14)).onTrue(new InstantCommand(() -> {
          LEDCalls.ON.cancel();
          LEDCalls.AMPLIFY_BLUE.cancel();
          LEDCalls.AMPLIFY_RED.activate();
        }));
        new Trigger(() -> buttonBox.getRawButton(13)).onTrue(new InstantCommand(() -> {
          LEDCalls.AMPLIFY_BLUE.cancel();
          LEDCalls.AMPLIFY_RED.cancel();
          LEDCalls.ON.activate();
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
        gyro,
        () -> driverController.getLeftY(), // fwd
        () -> driverController.getLeftX(), // str
        () -> driverController.getRightX(), // rcw
        new Trigger(() -> driverController.getBButton()), // resetPose
        new Trigger(() -> driverController.getAButton()), // flipMode
        new Trigger(() -> driverController.getYButton()) // lock rotation
    );

    drivetrain.setDefaultCommand(drivetrainDefault);

    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("One Piece", Autos.onePiece(superstructure, intake));
    autoChooser.addOption("Two Piece", Autos.twoPiece(drivetrain, superstructure, intake));
    autoChooser.addOption("Two piece amp side", Autos.twoPieceSide(drivetrain, superstructure, intake));
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
    // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        // drivetrain.getPose(),
        // new Pose2d(2, 6, drivetrain.getPose().getRotation())
    // );

    // PathPlannerPath path = new PathPlannerPath(
        // bezierPoints,
        // new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
        // new GoalEndState(0.0, drivetrain.getPose().getRotation())
    // );
    // path.preventFlipping = true;
    // PPLibTelemetry.setCurrentPath(path);
    // An example command will be run in autonomous
    return true ? autoChooser.getSelected()
      : new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece"));
      // return new SequentialCommandGroup(
        // new InstantCommand(() -> PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("test path"))),
        // new ParallelRaceGroup(
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece Side"))
          // new InstantCommand(() -> PPLibTelemetry.setCurrentPose(drivetrain.getPose()))).repeatedly()
      // );
      // return new ParallelCommandGroup(
      // new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
        // }),
        // new WaitUntilCommand(intake::atSetpoint),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
        // new ParallelCommandGroup(
            // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece Side")),
          // new SequentialCommandGroup(
            // new WaitCommand(6),
            // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
            // new WaitCommand(1.5),
            // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
            // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
          // )
        // )
      // )
    // );
    // return Autos.exampleAuto(m_exampleSubsystem);
      // new SequentialCommandGroup(
        // new ParallelRaceGroup(
            // new InstantCommand(() -> drivetrain.drive(new ChassisSpeeds(0, 0.2, 0))).repeatedly(),
            // new WaitCommand(5)
        // ),
        // new InstantCommand(drivetrain::stop)
      // );
    // return autoChooser.getSelected();
  }

  /** Robot periodic method. */
  public void robotPeriodic() {
    // PPLibTelemetry.setCurrentPose(drivetrain.getPose());
    // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("test path"));
    if (hardware == Hardware.HYPERION) {
      buttonBox.sendMessage();
    }
  }

  public void teleopPeriodic() {}

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
