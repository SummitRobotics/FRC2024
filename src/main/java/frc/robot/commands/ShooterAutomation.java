package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SuperstructureDefault.StateChangeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;

public class ShooterAutomation extends Command {

  private Translation2d SPEAKER_POSE;
  private final double speakerHeight = 2.032 + 0.3; // in meters
  private final double shooterInitialHeight = 0.36; // ground to shooter when elevator is collapsed fully
  private final double elevatorHighMeters = 0.78;
  private final double elevatorHighEncoder = 7.0;
  private final double pivotEncoderZero = 0.581 * 2 * Math.PI;
  private final double minAngleUp = 0.0544 * 2 * Math.PI - pivotEncoderZero; // What can we safely pivot to in high or low positions?
  private final double maxAngleUp = 0.7 * 2 * Math.PI - pivotEncoderZero;
  private final double minAngleDown = 0.0544 * 2 * Math.PI - pivotEncoderZero;
  private final double maxAngleDown = 0.665 * 2 * Math.PI - pivotEncoderZero;
  private final double spoolTime = 1.8;
  private final double feedTime = 0.4;
  private final double compensateForDistance = 0.05;
  private final double compensateForMovement = 0.0; //1.5 / 15.75; // in seconds per meter
  private final double MAX_SPEED;
  private Swerve drivetrain;
  private Superstructure superstructure;
  private Intake intake;
  // private PIDController pid = new PIDController(1, 0, 0.8);
  private PIDController pid = new PIDController(6, 0.01, 0);
  private Timer spoolTimer = new Timer();
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  SlewRateLimiter fwdLimiter;
  SlewRateLimiter strLimiter;
  private boolean isSplining = false;

  public ShooterAutomation(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    var alliance = DriverStation.getAlliance();
    SPEAKER_POSE = alliance.isPresent() && alliance.get() == Alliance.Red ? new Translation2d(16.5, 5.5) : new Translation2d(0, 5.5);
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.intake = intake;
    MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    fwd = () -> 0;
    str = () -> 0;
    this.fwdLimiter = new SlewRateLimiter(7.5);
    this.strLimiter = new SlewRateLimiter(7.5);
    addRequirements(drivetrain);
  }

  public ShooterAutomation(Swerve drivetrain, Superstructure superstructure, Intake intake, DoubleSupplier fwd, DoubleSupplier str) {
    var alliance = DriverStation.getAlliance();
    SPEAKER_POSE = alliance.isPresent() && alliance.get() == Alliance.Red ? new Translation2d(16.5, 5.5) : new Translation2d(0, 5.5);
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.intake = intake;
    this.fwd = fwd;
    this.str = str;
    MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    this.fwdLimiter = new SlewRateLimiter(7.5);
    this.strLimiter = new SlewRateLimiter(7.5);
    addRequirements(drivetrain);
  }

  public ShooterAutomation(Swerve drivetrain, Superstructure superstructure, Intake intake, boolean isSplining) {
    var alliance = DriverStation.getAlliance();
    SPEAKER_POSE = alliance.isPresent() && alliance.get() == Alliance.Red ? new Translation2d(16.5, 5.5) : new Translation2d(0, 5.5);
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.intake = intake;
    MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    fwd = () -> 0;
    str = () -> 0;
    this.fwdLimiter = new SlewRateLimiter(7.5);
    this.strLimiter = new SlewRateLimiter(7.5);
    this.isSplining = isSplining;
    if (!isSplining) {
      addRequirements(drivetrain);
    }
  }

  @Override
  public void initialize() {
  // TODO - retool after intake change
    // intake.setState(DriverStation.isAutonomous() ? IntakeState.DOWN : IntakeState.MID);
    spoolTimer.restart();
    // if (superstructure.getState() == SuperstructureState.IDLE) {
      // Superstructure.variableIndexer = 0;
    // } else {
      // Superstructure.variableIndexer = 0.17;
    // }
  }

  @Override
  public void execute() {
    // This makes more sense if you assume that the speaker is moving and the robot is stationary.
    // Align to speaker with intake out / shooter in; makeAngleContinuous to rotate around the right direction just like for swerve module rotations
    double distance = Math.sqrt(Math.pow(SPEAKER_POSE.getY() - drivetrain.getPose().getY(), 2)
        + Math.pow(SPEAKER_POSE.getX() - drivetrain.getPose().getX(), 2));
    Translation2d speakerPose = SPEAKER_POSE.plus(new Translation2d(drivetrain.getCurrentVelocity().vxMetersPerSecond * compensateForMovement * distance,
      drivetrain.getCurrentVelocity().vyMetersPerSecond * compensateForMovement * distance));

    // Rotate the speaker pose along an arc around the robot based on the angular velocity.
    // This is from https://math.stackexchange.com/questions/103202/calculating-the-position-of-a-point-along-an-arc.
    // The equation is like rotating around the unit circle, but scaled for the actual distance and translated so the bot pose is the origin.
    double theta = compensateForDistance * distance * drivetrain.getCurrentVelocity().omegaRadiansPerSecond;
    speakerPose = new Translation2d(
      drivetrain.getPose().getX() + (speakerPose.getX() - drivetrain.getPose().getX()) * Math.cos(theta)
        + (drivetrain.getPose().getY() - speakerPose.getY()) * Math.sin(theta),
      drivetrain.getPose().getY() + (speakerPose.getY() - drivetrain.getPose().getY()) * Math.cos(theta)
        + (drivetrain.getPose().getX() - speakerPose.getX()) * Math.sin(theta)
    );

    // Calculate the angle from speaker to robot, between -pi and pi. Positive angle is CCW from speaker towards x-origin.
    double drivetrainAngle = Math.atan2(speakerPose.getY() - drivetrain.getPose().getY(), speakerPose.getX() - drivetrain.getPose().getX());

    // Calculate angle difference between robot and speaker, we'll drive this to 0 in the PID controllers.
    // NOTE: the robot shoots from behind, so rotate by 180 degrees.
    double angleDiff = Rotation2d.fromRadians(drivetrainAngle).minus(drivetrain.getPose().getRotation().plus(Rotation2d.fromDegrees(180))).getRadians();
    System.out.println("Angle diff:" + angleDiff);

    var alliance = DriverStation.getAlliance();
    if (!isSplining) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        fwdLimiter.calculate(Math.copySign(Math.pow(fwd.getAsDouble(), 2),
          fwd.getAsDouble()) * MAX_SPEED / 4),
        strLimiter.calculate(Math.copySign(Math.pow(str.getAsDouble(), 2),
          str.getAsDouble()) * MAX_SPEED / 4),
        -pid.calculate(angleDiff)
      ), alliance.isPresent() && alliance.get() == Alliance.Red ? drivetrain.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)) : drivetrain.getPose().getRotation()));
    } else {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        drivetrain.getCurrentVelocity().vxMetersPerSecond,
        drivetrain.getCurrentVelocity().vyMetersPerSecond,
        -pid.calculate(angleDiff)
      ), alliance.isPresent() && alliance.get() == Alliance.Red ? drivetrain.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)) : drivetrain.getPose().getRotation()));
    }

    // TODO - retool after intake change

    if (true) {//Intake.pivot.getEncoder().getPosition() < 29/* && intake.getState() == IntakeState.MID*/) {
      if (superstructure.getState() != SuperstructureState.VARIABLE_READY && superstructure.getState() != SuperstructureState.VARIABLE_GO) {
        superstructure.setState(SuperstructureState.VARIABLE_READY);
      }
      // Right triangle with speaker as opposite and overhead field distance as adjacent
      // Check if shooting without moving elevator is possible
      double elevatorHeight = shooterInitialHeight;
      double overheadDistance = Math.sqrt(Math.pow(speakerPose.getY() - drivetrain.getPose().getY(), 2)
        + Math.pow(speakerPose.getX() - drivetrain.getPose().getX(), 2));
      double shootAngle = Math.atan2(speakerHeight - elevatorHeight, overheadDistance) + overheadDistance * Math.pow(compensateForDistance, 2);
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

      // if (superstructure.getState() != SuperstructureState.VARIABLE_GO && Superstructure.shooter.getToF()) {
        // Superstructure.variableIndexer = 0;
      // }

      // Wait until angle is within 5 degrees of target
      if (Math.abs(angleDiff) < Units.degreesToRadians(5)) {
        // Wait until elevator/pivot are at setpoints, and spooled up to shoot
        if (superstructure.atSetpoint() && spoolTimer.get() > spoolTime) {
          superstructure.setState(SuperstructureState.VARIABLE_GO);
          // Superstructure.variableIndexer = 0.8;
          spoolTimer.restart();
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return superstructure.getState() == SuperstructureState.VARIABLE_GO && spoolTimer.get() > feedTime;
  }

  @Override
  public void end(final boolean interrupted) {
    if (DriverStation.isAutonomous()) {
      superstructure.setState(SuperstructureState.RECEIVE);
    } else {
      CommandScheduler.getInstance().schedule(new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE));
    }
    drivetrain.stop();
  }
}
