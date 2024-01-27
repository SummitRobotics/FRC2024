package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;

/** A swerve drivetrain subsystem will extend this class. */
public abstract class Swerve extends SubsystemBase {
  public abstract SwerveConstellation getConstellation();

  public abstract Rotation2d getGyroscopeRotation();

  private boolean fieldOriented = true;

  protected ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  protected Translation2d rotationPoint = new Translation2d();
  protected Field2d field2d = new Field2d();

  private SwerveDrivePoseEstimator poseEstimator;

  protected SwerveDrivePoseEstimator getPoseEstimator() {
    if (poseEstimator == null) {
      // Rotate because forward for swerve modules does not coincide with forward for odometry
      SwerveModulePosition[] rotatedPoses = new SwerveModulePosition[
          getConstellation().modulePositions().length];
      for (int i = 0; i < getConstellation().modulePositions().length; i++) {
        rotatedPoses[i] = new SwerveModulePosition(getConstellation()
            .modulePositions()[i].distanceMeters, new Rotation2d(getConstellation()
            .modulePositions()[i].angle.getRadians() - Math.PI / 2));
      }
      poseEstimator = new SwerveDrivePoseEstimator(
        getConstellation().kinematics,
        getGyroscopeRotation(),
        rotatedPoses,
          new Pose2d(),
        VecBuilder.fill(0.02, 0.02, 0.01),
        VecBuilder.fill(0.1, 0.1, 0.01)
      );
    }
    return poseEstimator;
  }

  public Pose2d getPose() {
    return getPoseEstimator().getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    getPoseEstimator().resetPosition(getGyroscopeRotation(),
        getConstellation().modulePositions(), pose);
  }

  /** Set the drivetrain to move at a particular ChassisSpeed. */
  public void drive(ChassisSpeeds chassisSpeeds) {
    // These need to be flipped in order for odometry to work.
    // This might have something to do with robot pose coordinates not
    // being the same as swerve module-space coordinates.
    this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond,
    -chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    this.rotationPoint = new Translation2d();
  }

  /** Sets the drivetrain to move at a particular ChassisSpeed. */
  public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotationPoint) {
    this.chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vyMetersPerSecond,
      -chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    this.rotationPoint = rotationPoint;
  }

  public ChassisSpeeds getCurrentVelocity() {
    return getConstellation().chassisSpeeds();
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveConstellation constellation = getConstellation();
    // Rotate because forward for swerve modules does not coincide with forward for odometry
    SwerveModulePosition[] rotatedPoses = new SwerveModulePosition[
      constellation.modulePositions().length];
    for (int i = 0; i < constellation.modulePositions().length; i++) {
      rotatedPoses[i] = new SwerveModulePosition(constellation.modulePositions()[i]
        .distanceMeters, new Rotation2d(constellation.modulePositions()[i]
        .angle.getRadians() - Math.PI / 2));
    }
    getPoseEstimator().update(getGyroscopeRotation(), rotatedPoses);
    field2d.setRobotPose(getPoseEstimator().getEstimatedPosition());
    if (
        Functions.withinTolerance(chassisSpeeds.vxMetersPerSecond, 0, 0.01)
        && Functions.withinTolerance(chassisSpeeds.vyMetersPerSecond, 0, 0.01)
        && Functions.withinTolerance(chassisSpeeds.omegaRadiansPerSecond, 0, 0.01)
    ) {
      constellation.stopModules();
    } else {
      constellation.setModuleStates(chassisSpeeds, rotationPoint);
    }
    constellation.recalibrate();

    // AprilTag odometry
    // poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("Pipeline 1"),
    // LimelightHelpers.getLatestResults("Pipeline 1").targetingResults.timestamp_RIOFPGA_capture);
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
    builder.addDoubleProperty("Odometry Heading (Rads)",
        () -> getPose().getRotation().getRadians(), null);
    builder.addDoubleProperty("Velocity X", () -> getCurrentVelocity().vxMetersPerSecond, null);
    builder.addDoubleProperty("Velocity Y", () -> getCurrentVelocity().vyMetersPerSecond, null);
    builder.addDoubleProperty("Velocity Heading (Deg)",
        () -> getCurrentVelocity().omegaRadiansPerSecond * 180 / Math.PI, null);
    builder.addBooleanProperty("Field Oriented", () -> fieldOriented, null);
    SmartDashboard.putData("Field", field2d);
  }
}
