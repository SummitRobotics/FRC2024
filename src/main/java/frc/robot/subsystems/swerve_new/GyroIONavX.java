package frc.robot.subsystems.swerve_new;

import java.util.OptionalDouble;
import java.util.Queue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  // Hopefully this byte cast is okay - the NavX library had better process it as unsigned 
  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) Module.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX(boolean phoenixDrive) {
    if (phoenixDrive) {
      // TODO - address this
      throw new IllegalStateException("NavX is not currently supported alongside the Phoenix odometry thread.");
    } else {
      yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> OptionalDouble.of(navx.getAngle()));
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true; // Is there a way to read this?
    inputs.yawPosition = navx.getRotation2d().unaryMinus(); // Mounted upside-down
    inputs.yawVelocityRadPerSec = -navx.getRate();
    inputs.odometryYawTimestamps =
      yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
      yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(value))
        .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void reset() {
    navx.reset();
  }
}
