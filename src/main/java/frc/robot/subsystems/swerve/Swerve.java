package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.Results;

/** A swerve drivetrain subsystem will extend this class. */
public abstract class Swerve extends SubsystemBase {
  public abstract SwerveConstellation getConstellation();

  public abstract Rotation2d getGyroscopeRotation();
  public abstract double getGyroscopeAngularVelocity();

  private boolean fieldOriented = true;
  private final String[] limelightNames = {"limelight-lime", "limelight-orange"};
  // private Timer timer = new Timer();

  protected ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  protected Translation2d rotationPoint = new Translation2d();
  protected Field2d field2d = new Field2d();
  private Field2d limelightField = new Field2d();

  private SwerveDrivePoseEstimator poseEstimator;

  protected SwerveDrivePoseEstimator getPoseEstimator() {
    if (poseEstimator == null) {
      poseEstimator = new SwerveDrivePoseEstimator(
        getConstellation().kinematics,
        getGyroscopeRotation(),
        getConstellation().modulePositions(),
          new Pose2d(0, 0, new Rotation2d(0)),
        VecBuilder.fill(0.0025, 0.0025, 0.00125),
        VecBuilder.fill(0.72, 0.72, 0.36)
      );
    }
    return poseEstimator;
  }

  public Pose2d getPose() {
    return getPoseEstimator().getEstimatedPosition();
  }

  // public Pose2d getPoseFlipped() {
    // return getPoseEstimator().getEstimatedPosition().rotateBy(Rotation2d.fromRadians(Math.PI));
  // }

  public void setPose(Pose2d pose) {
    getPoseEstimator().resetPosition(getGyroscopeRotation(),
        getConstellation().modulePositions(), pose);
  }

  /** Set the drivetrain to move at a particular ChassisSpeed. */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
    this.rotationPoint = new Translation2d();
  }

  /** Sets the drivetrain to move at a particular ChassisSpeed. */
  public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotationPoint) {
    this.chassisSpeeds = chassisSpeeds;
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
    // timer.reset();
    // timer.start();
    SwerveConstellation constellation = getConstellation();
    // System.out.println("Line 102: " + timer.get());
    // System.out.println("Line 111: " + timer.get());
    getPoseEstimator().update(getGyroscopeRotation(), constellation.modulePositions());
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
    // System.out.println("Line 123: " + timer.get());
    constellation.recalibrate();
    // System.out.println("Line 125: " + timer.get());
    // AprilTag odometry
    for (String limelightName : limelightNames) {
      Results llResults = LimelightHelpers.getLatestResults(limelightName).targetingResults;
      Pose2d botPose = llResults.getBotPose2d_wpiBlue();
      limelightField.setRobotPose(botPose);
      if (llResults.valid && botPose.getX() != 0 && botPose.getY() != 0) {
        poseEstimator.addVisionMeasurement(new Pose2d(botPose.getX()/* + 16.541748984 / 2*/,
            botPose.getY() /*+ 8.21055 / 2*/, botPose.getRotation()),
            Timer.getFPGATimestamp() - llResults.latency_pipeline / 1000.0
            - llResults.latency_capture / 1000.0);
      }
    }
  }

  public void stop() {
    this.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
    // builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
    // builder.addDoubleProperty("Odometry Heading (Rads)",
        // () -> getPose().getRotation().getRadians(), null);
    // builder.addDoubleProperty("Velocity X", () -> getCurrentVelocity().vxMetersPerSecond, null);
    // builder.addDoubleProperty("Velocity Y", () -> getCurrentVelocity().vyMetersPerSecond, null);
    // builder.addDoubleProperty("Velocity Heading (Deg)",
        // () -> getCurrentVelocity().omegaRadiansPerSecond * 180 / Math.PI, null);
    builder.addBooleanProperty("Field Oriented", () -> fieldOriented, null);
    SmartDashboard.putData("Field", field2d);
    SmartDashboard.putData("Limelight Pose", limelightField);
  }
}
