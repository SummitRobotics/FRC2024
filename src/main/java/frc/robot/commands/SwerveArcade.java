package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.DoubleSupplier;

/** Default drive command for the swerve. */
public class SwerveArcade extends Command {
  Swerve drivetrain;
  Superstructure superstructure;
  // Intake intake;
  AHRS gyro;
  DoubleSupplier fwd;
  DoubleSupplier str;
  DoubleSupplier rcw;
  RisingEdgeTrigger resetPose;
  RisingEdgeTrigger flipMode;
  Trigger lockRotation;
  SlewRateLimiter fwdLimiter;
  SlewRateLimiter strLimiter;

  final double MAX_SPEED;
  boolean fieldOriented = true;
  // boolean rotationLocked = false;
  PIDController rotLockController = new PIDController(0.05, 0, 0.01);

  /** Creates a new ArcadeDrive. */
  public SwerveArcade(
      Swerve drivetrain,
      Superstructure superstructure,
      // Intake intake,
      AHRS gyro,
      DoubleSupplier fwd,
      DoubleSupplier str,
      DoubleSupplier rcw,
      Trigger resetPose,
      Trigger flipMode,
      Trigger lockRotation
  ) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    // this.intake = intake;
    this.gyro = gyro;
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;

    this.resetPose = new RisingEdgeTrigger(resetPose);
    this.flipMode = new RisingEdgeTrigger(flipMode);
    this.lockRotation = lockRotation;

    this.fwdLimiter = new SlewRateLimiter(5);
    this.strLimiter = new SlewRateLimiter(5);

    MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    // rotLockController.setTolerance(3);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    if (!fieldOriented && superstructure.getState() == SuperstructureState.IDLE) fieldOriented = true;

    if (flipMode.get()) {
      fieldOriented = !fieldOriented;
      drivetrain.setFieldOriented(fieldOriented);
    }
    // if (lockRotation.get()) {
      // rotationLocked = !rotationLocked; 
    // }
    if (resetPose.get()) {
      // drivetrain.setPose(new Pose2d());
      // gyro.calibrate();
      gyro.reset();
      // Sets drivetrain back to 0, reducing acumulated error
      drivetrain.setPose(new Pose2d(0, 0, new Rotation2d(0)));
    }

    double turnVal = 0;

    if (Math.abs(rcw.getAsDouble()) > 0.845) {
      turnVal = Math.pow(rcw.getAsDouble(), 3) * 0.7;
    } else {
      turnVal = 0.5 * rcw.getAsDouble();
    }

    ChassisSpeeds speed = new ChassisSpeeds();
    // Field oriented
    if (!lockRotation.getAsBoolean() && fieldOriented) {
      speed = new ChassisSpeeds(
          -fwdLimiter.calculate(Math.copySign(Math.pow(fwd.getAsDouble(), 5),
            fwd.getAsDouble()) * MAX_SPEED / 4),
          -strLimiter.calculate(Math.copySign(Math.pow(str.getAsDouble(), 5),
            str.getAsDouble()) * MAX_SPEED / 4),
          -turnVal * 20
      );
    }

    // Robot oriented
    if (!fieldOriented) {
      speed = new ChassisSpeeds(
          -fwdLimiter.calculate(Math.copySign(Math.pow(fwd.getAsDouble(), 5),
            fwd.getAsDouble()) * MAX_SPEED / 4),
          -strLimiter.calculate(Math.copySign(Math.pow(str.getAsDouble(), 5),
            str.getAsDouble()) * MAX_SPEED / 4),
          -turnVal * 20
      );
    }
    
    if (lockRotation.getAsBoolean()) {
      speed = new ChassisSpeeds(
          fwdLimiter.calculate(Math.copySign(Math.pow(fwd.getAsDouble(), 2),
            fwd.getAsDouble()) * MAX_SPEED / 4),
          strLimiter.calculate(Math.copySign(Math.pow(str.getAsDouble(), 2),
            str.getAsDouble()) * MAX_SPEED / 4),
          // strLimiter.calculate(str.getAsDouble() * MAX_SPEED / 4),
          -rotLockController.calculate(LimelightHelpers.getTX("limelight"))
      );
    }

    if (fieldOriented) {
      // drivetrain.drive(ChassisSpeeds
          // .fromFieldRelativeSpeeds(speed, drivetrain.getPose().getRotation()
          // .plus(Rotation2d.fromDegrees(drivetrain.getGyroscopeAngularVelocity()
          // * DrivetrainConstants.ANGULAR_VELOCITY_COEFFICIENT))));
      // System.out.println("Rotation: " + drivetrain.getPose().getRotation());
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        drivetrain.drive(ChassisSpeeds
            .fromFieldRelativeSpeeds(speed, alliance.get() == Alliance.Blue ? drivetrain.getPose().getRotation()
            : drivetrain.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))));
      } else {
        drivetrain.drive(ChassisSpeeds
            .fromFieldRelativeSpeeds(speed, drivetrain.getPose().getRotation()));
      }
    } else {
      drivetrain.drive(speed);
    }
  }

  @Override
  public void end(final boolean interrupted) {
    drivetrain.stop();
  }
}
