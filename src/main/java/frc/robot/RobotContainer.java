// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FollowPathPlannerTrajectory;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.SwerveArcade;
import frc.robot.oi.Controller;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Drivetrain;

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
  // private final Superstructure superstructure = new Superstructure();
  private final Intake intake = new Intake();
  // private final Climb climb = new Climb();

  // TODO - ports
  private final Controller driverController =
      new Controller(0);

  // private final ButtonBox buttonBox =
      // new ButtonBox(0);
  private final Controller gunnerController =
      new Controller(1);
  private final GenericHID buttonBox = new GenericHID(2);

  private final SwerveArcade drivetrainDefault = new SwerveArcade(
      drivetrain,
      gyro,
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      new Trigger(() -> driverController.getBButton()),
      new Trigger(() -> driverController.getAButton()),
      new Trigger(() -> driverController.getYButton())
  );


  // private final SuperstructureDefault superstructureDefault = new SuperstructureDefault(
      // superstructure,
      // intake,
      // new Trigger(() -> buttonBox.getRawButton(1)),
      // new Trigger(() -> buttonBox.getRawButton(2)),
      // new Trigger(() -> buttonBox.getRawButton(3)),
      // new Trigger(() -> buttonBox.getRawButton(4)),
      // new Trigger(() -> buttonBox.getRawButton(5)),
      // () -> buttonBox.getRawAxis(0),
      // () -> buttonBox.getRawAxis(1),
      // () -> buttonBox.getRawAxis(2),
      // () -> buttonBox.getRawAxis(3)
  // );

  private final IntakeDefault intakeDefault = new IntakeDefault(
      intake,
      // superstructure,
      new Trigger(() -> gunnerController.getYButton()),
      new Trigger(() -> driverController.getXButton()),
      () -> gunnerController.getLeftX(),
      // () -> gunnerController.getLeftY(),
      () -> gunnerController.getRightX()
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // superstructure.setDefaultCommand(superstructureDefault);
    configureBindings();
    // autoChooser.setDefaultOption("Test", new FollowPathPlannerTrajectory(drivetrain, "test"));
    // SmartDashboard.putData("Drivetrain", drivetrain);
    // SmartDashboard.putData("Auto Choice", autoChooser);
    SmartDashboard.putData("Intake", intake);
    var a = new RisingEdgeTrigger(gunnerController::getXButton);
    var b = new RisingEdgeTrigger(gunnerController::getYButton);
    SmartDashboard.putData("Controller", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("Left X", () -> a.get(), null);     
            builder.addBooleanProperty("Left Y", () -> b.get(), null);
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
  }

  public void autonomousPeriodic() {
    drivetrain.drive(new ChassisSpeeds(0.2, 0, 0));
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
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
    // return autoChooser.getSelected();
  }

  public void robotPeriodic() {
    // PPLibTelemetry.setCurrentPose(drivetrain.getPose());
    // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("test"));
  }
}
