package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.Functions;

public class ShooterAutomation extends Command {

  private Translation2d SPEAKER_POSE;
  // TODO - tune / rename final vars to match conventions
  private final double speakerHeight = 2.032 + 0.2; // in meters
  private final double shooterInitialHeight = 0.36; // ground to shooter when elevator is collapsed fully
  private final double elevatorHighMeters = 0.78;
  private final double elevatorHighEncoder = 7.0;
  private final double pivotEncoderZero = 0.581 * 2 * Math.PI;
  private final double minAngleUp = 0.0544 * 2 * Math.PI - pivotEncoderZero; // What can we safely pivot to in high or low positions?
  private final double maxAngleUp = 0.7 * 2 * Math.PI - pivotEncoderZero;
  private final double minAngleDown = 0.0544 * 2 * Math.PI - pivotEncoderZero;
  private final double maxAngleDown = 0.638 * 2 * Math.PI - pivotEncoderZero;
  private final double spoolTime = 1.6;
  private final double feedTime = 0.75;
  private final double compensateForDistance = 0.044;
  private final double compensateForMovement = 1.5 / 15.75; // in seconds per meter
  private final double MAX_SPEED;
  private Swerve drivetrain;
  private Superstructure superstructure;
  private Intake intake;
  // private PIDController pid = new PIDController(1, 0, 0.8);
  private PIDController pid = new PIDController(0.25, 0, 0);
  private Timer spoolTimer = new Timer();
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private double fwdSet = 0;
  private double strSet = 0;
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
    addRequirements(drivetrain);
  }

  public void setFwdAndStr(double fwd, double str) {
    fwdSet = fwd;
    strSet = str;
  }

  public static Command splineWhileShooting(Swerve drivetrain, Superstructure superstructure, Intake intake, PathPlannerPath path) {
    ShooterAutomation shootCommand = new ShooterAutomation(drivetrain, superstructure, intake, true);
    shootCommand.initialize();
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        new InstantCommand(() -> PPLibTelemetry.setCurrentPath(path)),
        new FollowPathHolonomic(
          path,
          drivetrain::getPose,
          drivetrain.getConstellation()::chassisSpeeds,
          (ChassisSpeeds speeds) -> {
            shootCommand.setFwdAndStr(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            shootCommand.execute();
          },
          new HolonomicPathFollowerConfig(
            new PIDConstants(2, 0, 0.05),
            new PIDConstants(2, 0, 0.05),
            4,
            0.45,
            new ReplanningConfig()
          ),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          drivetrain
        ),
        new InstantCommand(() -> shootCommand.end(false))
      )
    );
  }

  @Override
  public void initialize() {
    intake.setState(IntakeState.DOWN);
    spoolTimer.restart();
  }

  @Override
  public void execute() {
    // Align to speaker with intake out / shooter in; makeAngleContinuous to rotate around the right direction just like for swerve module rotations
    double distance = Math.sqrt(Math.pow(SPEAKER_POSE.getY() - drivetrain.getPose().getY(), 2)
        + Math.pow(SPEAKER_POSE.getX() - drivetrain.getPose().getX(), 2));
    Translation2d speakerPose = SPEAKER_POSE.plus(new Translation2d(drivetrain.getCurrentVelocity().vxMetersPerSecond * compensateForMovement * distance,
      drivetrain.getCurrentVelocity().vyMetersPerSecond * compensateForMovement * distance));

    // Calculate the angle from speaker to robot, between -pi and pi. Positive angle is CCW from speaker towards x-origin.
    double drivetrainAngle = Math.atan2(speakerPose.getY() - drivetrain.getPose().getY(), speakerPose.getX() - drivetrain.getPose().getX());

    // Calculate angle difference between robot and speaker, we'll drive this to 0 in the PID controllers.
    // NOTE: the robot is -180 pointing towards origin-x.
    double angleDiff = Functions.angleDifference(drivetrainAngle,
                                                 Functions.addAngles(drivetrain.getPose().getRotation().getRadians(), Math.PI));

    if (!isSplining) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        fwdLimiter.calculate(Math.copySign(Math.pow(fwd.getAsDouble(), 2),
          fwd.getAsDouble()) * MAX_SPEED / 4),
        strLimiter.calculate(Math.copySign(Math.pow(str.getAsDouble(), 2),
          str.getAsDouble()) * MAX_SPEED / 4),
        -pid.calculate(angleDiff)
      ), drivetrain.getPose().getRotation()));
    } else {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        fwdSet,
        strSet,
        -pid.calculate(angleDiff)
      ), drivetrain.getPose().getRotation()));
    }

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

      // Calculate absolute angle to target
      double angleToTarget = Functions.degreesToRadians(180) - Math.abs(drivetrainAngle - Functions.makeAngleContinuous(drivetrainAngle, drivetrain.getPose().getRotation().getRadians()));

      // Wait until angle is within 5 degrees of target
      if (angleToTarget < Functions.degreesToRadians(5)) {
        // Wait until elevator/pivot are at setpoints, and spooled up to shoot
        if (superstructure.atSetpoint() && spoolTimer.get() > spoolTime) {
          superstructure.setState(SuperstructureState.VARIABLE_GO);
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
    superstructure.setState(SuperstructureState.RECEIVE);
    drivetrain.stop();
  }
}
