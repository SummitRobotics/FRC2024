package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.Functions;

public class VariableShoot extends Command {

  private Translation2d speakerPose;
  // TODO - tune / rename IN_THIS_FORMAT
  private final double speakerHeight = 0; // in meters
  private final double shooterInitialHeight = 0;
  private final double elevatorMetersToEncoder = 0;
  private final double pivotRadiansToEncoder = 125 / (2 * Math.PI);
  private final double safeAngle = 0; // What can we safely pivot to without moving elevator / intake?
  private final double highShootHeight = 0; // Distance up for high shots
  private final double spoolTime = 1.5;
  private final double feedTime = 1.5;
  private Swerve drivetrain;
  private Superstructure superstructure;
  private Intake intake;
  private PIDController pid = new PIDController(1, 0, 0.01);
  private Timer spoolTimer = new Timer();

  public VariableShoot(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    speakerPose = DriverStation.getAlliance().get() == Alliance.Blue ? new Translation2d(0, 5.5) : new Translation2d(16.5, 5.5);
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.intake = intake;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    intake.setState(IntakeState.MID);
    spoolTimer.restart();
  }

  @Override
  public void execute() {
    // Align to speaker with intake out / shooter in; makeAngleContinuous to rotate around the right direction just like for swerve module rotations
    double drivetrainAngle = Math.PI + Math.atan2(speakerPose.getY() - drivetrain.getPose().getY(), speakerPose.getX() - drivetrain.getPose().getX());
    drivetrain.drive(new ChassisSpeeds(0, 0,
      -pid.calculate(Functions.makeAngleContinuous(drivetrainAngle, drivetrain.getPose().getRotation().getRadians()), drivetrainAngle)
    ));

    // TODO - change so this triggers once clear, not when at setpoint, to speed up
    if (intake.atSetpoint() && intake.getState() == IntakeState.MID) {
      if (superstructure.getState() != SuperstructureState.VARIABLE_READY || superstructure.getState() != SuperstructureState.VARIABLE_GO) {
        superstructure.setState(SuperstructureState.VARIABLE_READY);
      }
      // Right triangle with speaker as opposite and overhead field distance as adjacent
      // Check if shooting without moving elevator is possible
      double elevatorHeight = shooterInitialHeight;
      double shootAngle = Math.atan2(speakerHeight - elevatorHeight,
        Math.sqrt(Math.pow(speakerPose.getY() - drivetrain.getPose().getY(), 2) + Math.pow(speakerPose.getX() - drivetrain.getPose().getX(), 2)));
      // TODO - might need to be flipped for < depending on which way is positive
      if (shootAngle > safeAngle) {
        elevatorHeight = highShootHeight;
        shootAngle = Math.atan2(speakerHeight - elevatorHeight,
          Math.sqrt(Math.pow(speakerPose.getY() - drivetrain.getPose().getY(), 2) + Math.pow(speakerPose.getX() - drivetrain.getPose().getX(), 2)));
      }
      // TODO - add in corrective factors for air resistance / gravity as needed
      Superstructure.variableElevator = elevatorHeight * elevatorMetersToEncoder;
      Superstructure.variablePivot = shootAngle * pivotRadiansToEncoder;
      if (superstructure.atSetpoint() && spoolTimer.get() > spoolTime) {
        superstructure.setState(SuperstructureState.VARIABLE_GO);
        spoolTimer.restart();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == SuperstructureState.VARIABLE_GO && spoolTimer.get() > feedTime;
  }

  @Override
  public void end(final boolean interrupted) {
    CommandScheduler.getInstance().schedule(new SuperstructureDefault.StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE));
  }
}
