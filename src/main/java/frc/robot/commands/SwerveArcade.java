package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

/** Default drive command for the swerve. */
public class SwerveArcade extends Command {
  Swerve drivetrain;
  // Superstructure superstructure;
  // Intake intake;
  AHRS gyro;
  DoubleSupplier fwd;
  DoubleSupplier str;
  DoubleSupplier rcw;
  RisingEdgeTrigger resetPose;
  RisingEdgeTrigger flipMode;
  RisingEdgeTrigger rotaTrigger;
  SlewRateLimiter fwdLimiter;
  SlewRateLimiter strLimiter;

  final double MAX_SPEED;
  boolean fieldOriented = true;

  /** Creates a new ArcadeDrive. */
  public SwerveArcade(
      Swerve drivetrain,
      // Superstructure superstructure,
      // Intake intake,
      AHRS gyro,
      DoubleSupplier fwd,
      DoubleSupplier str,
      DoubleSupplier rcw,
      Trigger resetPose,
      Trigger flipMode,
      Trigger rotateTrigger
  ) {
    this.drivetrain = drivetrain;
    // this.superstructure = superstructure;
    // this.intake = intake;
    this.gyro = gyro;
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;

    this.resetPose = new RisingEdgeTrigger(resetPose);
    this.flipMode = new RisingEdgeTrigger(flipMode);
    this.rotaTrigger = new RisingEdgeTrigger(rotateTrigger);

    this.fwdLimiter = new SlewRateLimiter(4.5);
    this.strLimiter = new SlewRateLimiter(4.5);

    MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (/*superstructure.getState() != SuperstructureState.MANUAL_OVERRIDE
        && intake.getState() != IntakeState.MANUAL_OVERRIDE*/true) {
      if (flipMode.get()) {
        fieldOriented = !fieldOriented;
        drivetrain.setFieldOriented(fieldOriented);
      }
      if (resetPose.get()) {
        drivetrain.setPose(new Pose2d());
        // gyro.calibrate();
        gyro.reset();
        // Sets drivetrain back to 0, reducing acumulated error
        drivetrain.setPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));
      }

      double turnVal = 0;

      if (Math.abs(rcw.getAsDouble()) > 0.845) {
        turnVal = Math.pow(rcw.getAsDouble(), 3) * 0.7;
      } else {
        turnVal = 0.5 * rcw.getAsDouble();
      }

      ChassisSpeeds speed = new ChassisSpeeds(
          fwdLimiter.calculate(fwd.getAsDouble() * MAX_SPEED / 4),
          strLimiter.calculate(str.getAsDouble() * MAX_SPEED / 4),
          turnVal * 10
      );
      if (fieldOriented) {
        speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, drivetrain.getPose().getRotation());
        // System.out.println("Rotation: " + drivetrain.getPose().getRotation());
      }
      if (rotaTrigger.getTrigger().getAsBoolean()) {
        // System.out.println("HERE");
        drivetrain.drive(speed, new Translation2d(3, 0));
      } else {
        drivetrain.drive(speed);
      }
    } /* else {
      drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }*/
  }

  @Override
  public void end(final boolean interrupted) {
    drivetrain.stop();
  }
}
