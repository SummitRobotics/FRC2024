package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.swerve.SwerveModuleBuilder.SWERVE_MODULE_PRESETS;
import frc.robot.utilities.ConfigManager;

/** Represents the drivetrain subsystem. */
public class HyperionDrivetrain extends Swerve {
  private static final ConfigManager.PrefixedConfigAccessor config = ConfigManager.getInstance()
      .getPrefixedAccessor("DriveTrain.");
  private SwerveConstellation constellation;
  public SwerveModule mod0;
  public SwerveModule mod1;
  public SwerveModule mod2;
  public SwerveModule mod3;
  public AHRS gyro;
  public static final double DRIVE_P = config.getDouble("PID.P", 0.1);
  public static final double DRIVE_I = config.getDouble("PID.I", 0);
  public static final double DRIVE_D = config.getDouble("PID.D", 0);
  public static final double[] DRIVE_PID = new double[] { DRIVE_P, DRIVE_I, DRIVE_D };

  /** Creates a new Drivetrain. */
  public HyperionDrivetrain(AHRS gyro) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 3);

    // TODO - set values
    // front left
    mod0 = new SwerveModuleBuilder(new Translation2d(67.6 / 200, 58.7 / 200),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L3)
      .driveNEO1650(40).turnNEO1650(61)
      .CANCoder(2, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build(); //good
    // front right
    mod1 = new SwerveModuleBuilder(new Translation2d(-67.6 / 200, 58.7 / 200),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L3)
      .driveNEO1650(2).turnNEO1650(30)
      .CANCoder(1, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build(); //good
    // back right
    mod2 = new SwerveModuleBuilder(new Translation2d(67.6 / 200, -58.7 / 200),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L3)
      .driveNEO1650(11).turnNEO1650(60)
      .CANCoder(4, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build();
    // back left
    mod3 = new SwerveModuleBuilder(new Translation2d(-67.6 / 200, -58.7 / 200),
      SWERVE_MODULE_PRESETS.SDS_MK4i_L3)
      .driveNEO1650(62).turnNEO1650(50)
      .CANCoder(3, 0)
      .driveFeedforward(feedforward).drivePID(DRIVE_PID).build(); //good

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
