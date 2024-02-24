// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbDefault;
import frc.robot.commands.ExampleCommand;
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
import frc.robot.subsystems.swerve.Drivetrain;
import org.littletonrobotics.urcl.URCL;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AHRS gyro = new AHRS();
  private final Drivetrain drivetrain = new Drivetrain(gyro);
  // private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final Superstructure superstructure = new Superstructure();
  private final Intake intake = new Intake();
  // private final Climb climb = new Climb();
  // private final CANSparkMax indexer = new CANSparkMax(10, MotorType.kBrushless);
  private final PowerDistribution pdp = new PowerDistribution();

  // Instantiate USB devices
  private final Controller driverController = new Controller(OperatorConstants.kDriverControllerPort);
  private final Controller gunnerController = new Controller(OperatorConstants.kGunnerControllerPort);
  private final ButtonBox buttonBox = new ButtonBox(OperatorConstants.kButtonBoxPort);

  private final SwerveArcade drivetrainDefault = new SwerveArcade(
      drivetrain,
      gyro,
      () -> driverController.getLeftY(), // fwd
      () -> driverController.getLeftX(), // str
      () -> driverController.getRightX(), // rcw
      new Trigger(() -> driverController.getBButton()), // resetPose
      new Trigger(() -> driverController.getAButton()), // flipMode
      new Trigger(() -> driverController.getYButton()) // lock rotation
  );

  private final SuperstructureDefault superstructureDefault = new SuperstructureDefault(
      superstructure,
      intake,
      new Trigger(() -> buttonBox.getRawButton(1)), // receiveSupplier
      new Trigger(() -> buttonBox.getRawButton(2)), // ampSupplier
      new Trigger(() -> buttonBox.getRawButton(3)), // trapSupplier
      new Trigger(() -> buttonBox.getRawButton(4)), // shootSupplier
      new Trigger(() -> { // manualOverrideSupplier
        boolean value = gunnerController.getYButton();
        buttonBox.LED(ButtonBox.Button.MANUAL_OVERRIDE, value);
        return value;
      }),
      () -> -gunnerController.getLeftTrigger() + gunnerController.getRightTrigger(), // elevatorManualSupplier
      () -> gunnerController.getAButton() ? 1 : 0, // shooterManualSupplier
      () -> gunnerController.getRightY(), // indexerManualSupplier
      () -> gunnerController.getRightX(), // pivotManualSupplier
      new Trigger(() -> buttonBox.getRawButton(6))
  );

  private final IntakeDefault intakeDefault = new IntakeDefault(
      intake,
      superstructure,
      new Trigger(() -> gunnerController.getYButton()), // manualOverrideSupplier
      new Trigger(() -> driverController.getXButton()), // pivotUpandDown
      () -> -gunnerController.getLeftY(), // manualPivot
      // () -> gunnerController.getLeftY(),
      () -> gunnerController.getLeftX() // manualRoller
  );

  // private final ClimbDefault climbDefault = new ClimbDefault(
      // climb,
      // intake,
      // new Trigger(() -> gunnerController.getYButton()),
      // new Trigger(() -> false),
      // new Trigger(() -> gunnerController.getRightBumper()),
      // new Trigger(() -> gunnerController.getBButton()),
      // new Trigger(() -> gunnerController.getLeftBumper()),
      // new Trigger(() -> gunnerController.getXButton())  
  // );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // autoChooser.setDefaultOption("Test", new
    // FollowPathPlannerTrajectory(drivetrain, "test"));
    // SmartDashboard.putData("Drivetrain", drivetrain);
    // SmartDashboard.putData("Auto Choice", autoChooser);
    SmartDashboard.putData("Intake", intake);
    SmartDashboard.putData("Elevator / Shooter", superstructure);
    SmartDashboard.putData("Controller", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Left X", gunnerController::getXButton, null);
        builder.addBooleanProperty("Left Y", gunnerController::getYButton, null);
      }
    });
    SmartDashboard.putData("PDP", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current Draw", pdp::getTotalCurrent, null);
      }
    });
    // SmartDashboard.putData("Gyro", new Sendable() {
    // public void initSendable(SendableBuilder builder) {
    // builder.addFloatProperty("Pitch", gyro::getPitch, null);
    // builder.addFloatProperty("Yaw", gyro::getYaw, null);
    // builder.addFloatProperty("Roll", gyro::getRoll, null);
    // }
    // });
    drivetrain.setDefaultCommand(drivetrainDefault);
    intake.setDefaultCommand(intakeDefault);
    superstructure.setDefaultCommand(superstructureDefault);
    // climb.setDefaultCommand(climbDefault);

    // Just enable all lights for 500ms.
    // NOTE: this will override any other LED calls in the interim.
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

  public void autonomousPeriodic() {
    // drivetrain.drive(new ChassisSpeeds(0.2, 0, 0));
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
    // Intake recalibrate
    new Trigger(() -> driverController.getRightBumper()).onTrue(new InstantCommand(() -> {
      Intake.pivot.getEncoder().setPosition(Intake.DOWNPOSITION);
      intake.setState(IntakeState.DOWN);
    }));

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
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return new SequentialCommandGroup(
      Superstructure.elevator.routine.quasistatic(Direction.kForward),
      Superstructure.elevator.routine.quasistatic(Direction.kReverse),
      Superstructure.elevator.routine.dynamic(Direction.kForward),
      Superstructure.elevator.routine.dynamic(Direction.kReverse)
    );
    // return autoChooser.getSelected();
  }

  public void robotPeriodic() {
    // PPLibTelemetry.setCurrentPose(drivetrain.getPose());
    // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("test"));
    buttonBox.sendMessage();
  }

  public void teleopPeriodic() {
    // indexer.set(0.4 * gunnerController.getLeftTrigger()
    //- 0.4 * gunnerController.getRightTrigger());
  }

  public void robotInit() {
    DataLogManager.start();
    URCL.start();
  }
}
