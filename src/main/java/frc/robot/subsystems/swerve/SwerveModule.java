package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Represents an individual swerve module. */
public class SwerveModule implements Sendable {
  protected static void setModuleStates(SwerveModuleState[] states, SwerveModule... modules) {
    // First we need to check if there are the same number of modules and states
    if (modules.length != states.length) {
      throw new IllegalArgumentException("The number of modules and states must be the same");
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /** Gets states for modules. */
  public static SwerveModuleState[] computeModuleStates(SwerveModule... modules) {
    ArrayList<SwerveModuleState> states = new ArrayList<>();

    for (SwerveModule module : modules) {
      states.add(module.getState());
    }

    return states.toArray(new SwerveModuleState[0]);
  }

  /** Returns positions for modules. */
  public static SwerveModulePosition[] computeModulePositions(SwerveModule... modules) {
    ArrayList<SwerveModulePosition> positions = new ArrayList<>();

    for (SwerveModule module : modules) {
      positions.add(module.getPosition());
    }

    return positions.toArray(new SwerveModulePosition[0]);
  }

  protected static SwerveDriveKinematics computeKinematics(SwerveModule... modules) {
    ArrayList<Translation2d> locations = new ArrayList<>();

    for (SwerveModule module : modules) {
      locations.add(module.location);
    }

    return new SwerveDriveKinematics(locations.toArray(new Translation2d[0]));
  }

  /** Computes module states. */
  public static ChassisSpeeds computeChassisSpeeds(
      SwerveDriveKinematics kinematics, SwerveModule... modules) {
    return kinematics.toChassisSpeeds(computeModuleStates(modules));
  }

  protected static void recalibrate(SwerveModule... modules) {
    for (SwerveModule module : modules) {
      module.recalibrate();
    }
  }

  protected void recalibrate() {
    recalibrate.run();
  }

  private final Translation2d location;
  private final Supplier<Double> speedSupplier;
  private final Supplier<Rotation2d> angleSupplier;
  private final Supplier<Double> distanceSupplier;
  private final Consumer<Double> speedConsumer;
  private final Consumer<Rotation2d> angleConsumer;
  private SwerveModuleState targetState = new SwerveModuleState();

  private final Runnable recalibrate;

  public final double MAX_SPEED_METERS_PER_SECOND;

  private final Object driveMotor;
  private final Object turnMotor;

  protected SwerveModule(
      Translation2d location,
      Supplier<Double> speedSupplier,
      Supplier<Rotation2d> angleSupplier,
      Supplier<Double> distanceSupplier,
      Consumer<Double> speedConsumer,
      Consumer<Rotation2d> angleConsumer,
      Runnable recal,
      double maxSpeed,
      Object driveMotor,
      Object turnMotor
  ) {
    this.location = location;
    this.speedSupplier = speedSupplier;
    this.angleSupplier = angleSupplier;
    this.distanceSupplier = distanceSupplier;
    this.angleConsumer = angleConsumer;
    this.speedConsumer = speedConsumer;
    this.recalibrate = recal;
    this.MAX_SPEED_METERS_PER_SECOND = maxSpeed;
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(speedSupplier.get(), angleSupplier.get());
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distanceSupplier.get(), angleSupplier.get());
  }

  protected void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
    speedConsumer.accept(optimizedState.speedMetersPerSecond);
    angleConsumer.accept(optimizedState.angle);
    targetState = optimizedState;
  }

  public Translation2d getLocation() {
    return location;
  }


  /**
   * Gets the drive motor of this swerve module.
   * This will either be a CANSparkMax or a TalonFX.
   * Passing the wrong type will cause a ClassCastException, so use this method wisely.
   *
   * @param <T> The type of the motor controller object.
   * @return The drive motor.
   */
  @SuppressWarnings("unchecked")
  public <T> T getDriveMotor() {
    return (T) driveMotor;
  }

  /**
   * Gets the turn module of this swerve module.
   * This will either be a CANSparkMax or a TalonFX.
   * Passing the wrong type will cause a ClassCastException, so use this method wisely.
   *
   * @param <T> The type of the motor controller object.
   * @return The turn motor.
   */
  @SuppressWarnings("unchecked")
  public <T> T getTurnMotor() {
    return (T) turnMotor;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Speed (Real MPS)", speedSupplier::get, null);
    builder.addDoubleProperty("Angle (Real Deg)", () -> angleSupplier.get().getDegrees(), null);
    builder.addDoubleProperty("Angle (Real Rad)", () -> angleSupplier.get().getRadians(), null);
    builder.addDoubleProperty("Speed (Target MPS)", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Angle (Target Deg)", () -> targetState.angle.getDegrees(), null);
    builder.addDoubleProperty("Angle (Target Rad)", () -> targetState.angle.getRadians(), null);    
  }
}
