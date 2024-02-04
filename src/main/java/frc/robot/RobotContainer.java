// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveArcade;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AHRS gyro = new AHRS();
  private final Drivetrain drivetrain = new Drivetrain(gyro);
  // private final Superstructure superstructure = new Superstructure();
  // private final Intake intake = new Intake();
  // private final Climb climb = new Climb();

  // TODO - ports
  private final XboxController driverController =
      new XboxController(0);
  // private final XboxController gunnerController =
      // new XboxController(1);
  // private final GenericHID buttonBox = new GenericHID(0);

  private final SwerveArcade drivetrainDefault = new SwerveArcade(
      drivetrain,
      gyro,
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      new Trigger(() -> driverController.getPOV() == 0),
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

  // private final IntakeDefault intakeDefault = new IntakeDefault(
      // intake,
      // superstructure,
      // new Trigger(() -> buttonBox.getRawButton(0)),
      // new Trigger(() -> driverController.getBButton()),
      // () -> gunnerController.getLeftX(),
      // () -> gunnerController.getLeftY(),
      // () -> gunnerController.getRightX()
  // );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // superstructure.setDefaultCommand(superstructureDefault);
    configureBindings();
    SmartDashboard.putData("Drivetrain", drivetrain);
    SmartDashboard.putData("Controller", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Left X", driverController::getLeftX, null);     
            builder.addDoubleProperty("Left Y", driverController::getLeftY, null);
        }
    });
    drivetrain.setDefaultCommand(drivetrainDefault);
  }

  public void autonomousPeriodic() {
    drivetrain.drive(new ChassisSpeeds(0, 0.2, 0));
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
  }
}
