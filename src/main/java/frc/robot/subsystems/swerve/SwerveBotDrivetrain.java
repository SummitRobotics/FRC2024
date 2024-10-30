package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.swerve.SwerveModuleBuilder.SWERVE_MODULE_PRESETS;

/** Represents the drivetrain subsystem. */
public class SwerveBotDrivetrain extends Swerve {
  private SwerveConstellation constellation;
  public SwerveModule mod0;
  public SwerveModule mod1;
  public SwerveModule mod2;
  public SwerveModule mod3;
  public AHRS gyro;
  public static final double DRIVE_P = 0.1;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double[] DRIVE_PID = new double[] { DRIVE_P, DRIVE_I, DRIVE_D };

  /** Creates a new Drivetrain. */
  public SwerveBotDrivetrain(AHRS gyro) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 3);

    // front right - 2 - good
    mod0 = new SwerveModuleBuilder(new Translation2d(0.27 / 2, -0.27 / 2),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L2)
      .driveKraken(2).turnNEO1650(10)
      .CANCoder(31, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build();
    // front left - 1
    mod1 = new SwerveModuleBuilder(new Translation2d(0.27 / 2, 0.27 / 2),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L2)
      .driveKraken(1).turnNEO1650(8)
      .CANCoder(30, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build();
    // back right - 3
    mod2 = new SwerveModuleBuilder(new Translation2d(-0.27 / 2, -0.27 / 2),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L2)
      .driveKraken(3).turnNEO1650(12)
      .CANCoder(33, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build();
    // back left - 0
    mod3 = new SwerveModuleBuilder(new Translation2d(-0.27 / 2, 0.27 / 2),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L2)
      .driveKraken(0).turnNEO1650(2)
      .CANCoder(32, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build();
    
    constellation = new SwerveConstellation(mod0, mod1, mod2, mod3);
    this.gyro = gyro;
  }

  @Override
  public SwerveConstellation getConstellation() {
    return constellation;
  }

  @Override
  public Rotation2d getGyroscopeRotation() {
    return gyro.getRotation2d().unaryMinus();
  }

  @Override
  public double getGyroscopeAngularVelocity() {
    return gyro.getRate();
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // For AdvantageScope swerve visualizer; see https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/SWERVE.md.
    builder.addDoubleArrayProperty("Measured Module States", () -> new double[] {
        mod0.getState().angle.getRadians(), mod0.getState().speedMetersPerSecond,
        mod1.getState().angle.getRadians(), mod1.getState().speedMetersPerSecond,
        mod2.getState().angle.getRadians(), mod2.getState().speedMetersPerSecond,
        mod3.getState().angle.getRadians(), mod3.getState().speedMetersPerSecond,
    }, null);
    builder.addDoubleProperty("Upper Right Heading",
        () -> mod0.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Upper Left Heading",
        () -> mod1.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Bottom Right Heading",
        () -> mod2.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Botton Left Heading",
        () -> mod3.getState().angle.getRadians(), null);
    // builder.addDoubleArrayProperty("Target Module States", () -> new double[] {
    // mod0.getTargetState().angle.getRadians(), mod0.getTargetState().speedMetersPerSecond,
    // mod1.getTargetState().angle.getRadians(), mod1.getTargetState().speedMetersPerSecond,
    // mod2.getTargetState().angle.getRadians(), mod2.getTargetState().speedMetersPerSecond,
    // mod3.getTargetState().angle.getRadians(), mod3.getTargetState().speedMetersPerSecond
    // }, null);
  }
}
