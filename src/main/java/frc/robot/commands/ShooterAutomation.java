package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.Functions;

public class ShooterAutomation extends Command {

  private Translation2d speakerPose;
  // TODO - tune / rename final vars to match conventions
  private final double speakerHeight = 2.032; // in meters
  private final double shooterInitialHeight = 0.36; // ground to shooter when elevator is collapsed fully
  private final double elevatorHighMeters = 0.78;
  private final double elevatorHighEncoder = 7.0;
  private final double pivotEncoderZero = 0.581 * 2 * Math.PI;
  private final double minAngleUp = 0.0544 * 2 * Math.PI - pivotEncoderZero; // What can we safely pivot to in high or low positions?
  private final double maxAngleUp = 0.7 * 2 * Math.PI - pivotEncoderZero;
  private final double minAngleDown = 0.0544 * 2 * Math.PI - pivotEncoderZero;
  private final double maxAngleDown = 0.638 * 2 * Math.PI - pivotEncoderZero;
  private final double spoolTime = 2;
  private final double feedTime = 1.5;
  private final double compensateForDistance = 0.044;
  private Swerve drivetrain;
  private Superstructure superstructure;
  private Intake intake;
  // private PIDController pid = new PIDController(1, 0, 0.8);
  private PIDController pid = new PIDController(0.25, 0, 0);
  private Timer spoolTimer = new Timer();

  public ShooterAutomation(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    var alliance = DriverStation.getAlliance();
    speakerPose = alliance.isPresent() && alliance.get() == Alliance.Red ? new Translation2d(16.5, 5.5) : new Translation2d(0, 5.5);
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.intake = intake;
    // pid.setTolerance(90 * Math.PI / 180);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    intake.setState(IntakeState.DOWN);
    spoolTimer.restart();
  }

  @Override
  public void execute() {
    // Align to speaker with intake out / shooter in; makeAngleContinuous to rotate around the right direction just like for swerve module rotations
    double drivetrainAngle = Math.atan2(speakerPose.getY() - drivetrain.getPose().getY(), speakerPose.getX() - drivetrain.getPose().getX());
    drivetrain.drive(new ChassisSpeeds(0, 0,
      -pid.calculate(Functions.makeAngleContinuous(drivetrainAngle, drivetrain.getPose().getRotation().getRadians()), drivetrainAngle)
    ));

    // TODO - change so this triggers once clear, not when at setpoint, to speed up
    if (Intake.pivot.getEncoder().getPosition() < -29/* && intake.getState() == IntakeState.MID*/) {
      if (superstructure.getState() != SuperstructureState.VARIABLE_READY && superstructure.getState() != SuperstructureState.VARIABLE_GO) {
        superstructure.setState(SuperstructureState.VARIABLE_READY);
      }
      // Right triangle with speaker as opposite and overhead field distance as adjacent
      // Check if shooting without moving elevator is possible
      double elevatorHeight = shooterInitialHeight;
      double overheadDistance = Math.sqrt(Math.pow(speakerPose.getY() - drivetrain.getPose().getY(), 2)
        + Math.pow(speakerPose.getX() - drivetrain.getPose().getX(), 2));
      double shootAngle = Math.atan2(speakerHeight - elevatorHeight, overheadDistance) + overheadDistance * compensateForDistance;
      // // TODO - might need to be flipped for < depending on which way is positive
      if (minAngleDown > shootAngle || maxAngleDown < shootAngle) {
        elevatorHeight = elevatorHighMeters;
        shootAngle = Math.atan2(speakerHeight - elevatorHeight, overheadDistance);
        if (minAngleUp < shootAngle && maxAngleUp > shootAngle) {
          Superstructure.variableElevator = elevatorHighEncoder;
          Superstructure.variablePivot = shootAngle + pivotEncoderZero;
        }
      } else {
        Superstructure.variableElevator = 0;
        Superstructure.variablePivot = shootAngle + pivotEncoderZero;
      }
      
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
    superstructure.setState(SuperstructureState.RECEIVE);
    drivetrain.stop();
  }
}
