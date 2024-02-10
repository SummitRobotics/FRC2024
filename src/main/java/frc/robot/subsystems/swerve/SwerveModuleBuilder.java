package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Constructs an individual SwerveModule. */
public class SwerveModuleBuilder {

  /** PID values and physical constants for SDS Mk4i modules. */
  public static enum SWERVE_MODULE_PRESETS {
    SDS_MK4i_L1(8.14, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.0, 0.0, 0.0}, 0.533333333333),
    SDS_MK4i_L2(6.75, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.25, 0, 0.005}, 0.533333333333),
    SDS_MK4i_L3(6.12, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.25, 0.0, 0.005}, 0.533333333333);

    public final double driveGearRatio;
    public final double turnGearRatio;
    public final double wheelDiameter;
    public final double[] turnPID;
    public final double turnToDrive;

    SWERVE_MODULE_PRESETS(double driveGearRatio, double turnGearRatio, double wheelDiameter,
        double[] turnPID, double turnToDrive) {
      this.driveGearRatio = driveGearRatio;
      this.turnGearRatio = turnGearRatio;
      this.wheelDiameter = wheelDiameter;
      this.turnPID = turnPID;
      this.turnToDrive = turnToDrive;
    }
  }

  private Translation2d location;
  // A ratio of 2 means that the drive motor spins twice as fast as the wheel
  private double driveGearRatio = 0;
  // A ratio of 2 means that the turn motor spins twice as fast as the rotate wheel
  private double turnGearRatio = 0;
  private double wheelDiameter = 0; // In meters
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0);
  private double driveP = 0;
  private double driveI = 0;
  private double driveD = 0;
  private double turnP = 0;
  private double turnI = 0;
  private double turnD = 0;
  private double driveMotorMaxRPM = 0;
  // This is here because for some modules turning the turn motor also spins the wheel. 
  private double turnToDriveRatio = 0;
  private CANSparkMax sparkMaxDriveMotor;
  private CANSparkMax sparkMaxTurnMotor;
  private TalonFX falconDriveMotor;
  private TalonFX falconTurnMotor;
    
  // In radians the encoder reads positive is counterclockwise
  private Supplier<Double> turnEncoderAbsolute;

  private boolean built = false; // If the module has been built

  // The CANCoder angle is a discontinuous angle;
  // PID controllers don't like the sudden jump between 0 and 360.
  // Both angleConsumer and recalibrate convert to the version of
  // that angle within 180 of the current position.
  // Formula to eliminate jumps: (integer number of 360s to produce something
  // close to the current angle) * 360 + (smallest representation of current angle)
  private double makeAngleContinuous(double current, double target) {
    return Math.round((current - target % (2 * Math.PI))
      / (2 * Math.PI)) * 2 * Math.PI + target % (2 * Math.PI);
  }

  public SwerveModuleBuilder() {}

  public SwerveModuleBuilder(Translation2d location) {
    this.location = location;
  }

  /** Constructor. */
  public SwerveModuleBuilder(Translation2d location, SWERVE_MODULE_PRESETS preset) {
    this.location = location;
    this.driveGearRatio = preset.driveGearRatio;
    this.turnGearRatio = preset.turnGearRatio;
    this.wheelDiameter = preset.wheelDiameter;
    this.turnP = preset.turnPID[0];
    this.turnI = preset.turnPID[1];
    this.turnD = preset.turnPID[2];
    this.turnToDriveRatio = preset.turnToDrive;
  }

  public SwerveModuleBuilder location(Translation2d location) {
    this.location = location;
    return this;
  }

  public SwerveModuleBuilder driveGearRatio(double driveGearRatio) {
    this.driveGearRatio = driveGearRatio;
    return this;
  }

  public SwerveModuleBuilder turnGearRatio(double turnGearRatio) {
    this.turnGearRatio = turnGearRatio;
    return this;
  }
  
  public SwerveModuleBuilder wheelDiameter(double wheelDiameter) {
    this.wheelDiameter = wheelDiameter;
    return this;
  }
  
  public SwerveModuleBuilder driveFeedforward(SimpleMotorFeedforward driveFeedforward) {
    this.driveFeedforward = driveFeedforward;
    return this;
  }

  public SwerveModuleBuilder turnFeedforward(SimpleMotorFeedforward turnFeedforward) {
    this.turnFeedforward = turnFeedforward;
    return this;
  }

  /** Set drive PID values. */
  public SwerveModuleBuilder drivePID(double driveP, double driveI, double driveD) {
    this.driveP = driveP;
    this.driveI = driveI;
    this.driveD = driveD;
    return this;
  }

  /** Sets drive PID values. */
  public SwerveModuleBuilder drivePID(double[] pid) {
    if (pid.length != 3) {
      return this;
    }
    this.driveP = pid[0];
    this.driveI = pid[1];
    this.driveD = pid[2];
    return this;
  }
  
  /** Sets turn PID values. */
  public SwerveModuleBuilder turnPID(double turnP, double turnI, double turnD) {
    this.turnP = turnP;
    this.turnI = turnI;
    this.turnD = turnD;
    return this;
  }
  
  /** Sets turn PID values. */
  public SwerveModuleBuilder turnPID(double[] pid) {
    if (pid.length != 3) {
      return this;
    }
    this.turnP = pid[0];
    this.turnI = pid[1];
    this.turnD = pid[2];
    return this;
  }

  /** Sets a NEO 1650 drive motor. */
  public SwerveModuleBuilder driveNEO1650(int deviceID) {
    this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    this.driveMotorMaxRPM = 5_676;
    return this;
  }
  
  /** Sets a NEO 1650 turn motor. */
  public SwerveModuleBuilder turnNEO1650(int deviceID) {
    this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    return this;
  }

  /** Sets a Falcon 500 turn motor. */
  public SwerveModuleBuilder driveFalcon500(int deviceID) {
    this.falconDriveMotor = new TalonFX(deviceID);
    this.driveMotorMaxRPM = 6_380;
    return this;
  }
  
  public SwerveModuleBuilder turnFalcon500(int deviceID) {
    this.falconTurnMotor = new TalonFX(deviceID);
    return this;
  }
  
  /** Sets a drive NEO 550. */
  public SwerveModuleBuilder driveNEO550(int deviceID) {
    this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    this.driveMotorMaxRPM = 11_000;
    return this;
  }
  
  public SwerveModuleBuilder turnNEO550(int deviceID) {
    this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    return this;
  }
  
  public SwerveModuleBuilder turnEncoderAbsolute(Supplier<Double> turnEncoderAbsolute) {
    this.turnEncoderAbsolute = turnEncoderAbsolute;
    return this;
  }

  /** Sets a module's CANCoder by CAN ID and offset.
   */
  @SuppressWarnings("all") // canCoder causes a resource leak - doesn't seem to be a problem
  public SwerveModuleBuilder CANCoder(int deviceID, double offset) {
    CANcoder canCoder = new CANcoder(deviceID);
    // Keep things simple with only positive values, although -180 to 180 might also work.
    canCoder.getConfigurator().apply(new MagnetSensorConfigs()
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(Math.toDegrees(offset))
    );
    this.turnEncoderAbsolute = () -> {
      return 2 * Math.PI * canCoder.getAbsolutePosition().getValueAsDouble();
    };
    return this;
  }

  /** Checks if given all parameters and returns the SwerveModule. */
  public SwerveModule build() {
    if (built) {
      throw new IllegalStateException("SwerveModule has already been built");
    }
    if (location == null) {
      throw new IllegalStateException("Location has not been set");
    }
    if (driveGearRatio == 0) {
      throw new IllegalStateException("Drive gear ratio has not been set");
    }
    if (turnGearRatio == 0) {
      throw new IllegalStateException("Turn gear ratio has not been set");
    }
    if (wheelDiameter == 0) {
      throw new IllegalStateException("Wheel diameter has not been set");
    }
    if (driveMotorMaxRPM == 0) {
      throw new IllegalStateException("Drive motor max RPM has not been set");
    }
    if (sparkMaxDriveMotor == null && falconDriveMotor == null) {
      throw new IllegalStateException("Drive motor has not been set");
    }
    if (sparkMaxTurnMotor == null && falconTurnMotor == null) {
      throw new IllegalStateException("Turn motor has not been set");
    }
    if (turnEncoderAbsolute == null) {
      throw new IllegalStateException("Turn encoder has not been set");
    }
    if (sparkMaxDriveMotor != null && falconDriveMotor != null) {
      throw new IllegalStateException("Drive motor has been set twice");
    }
    if (sparkMaxTurnMotor != null && falconTurnMotor != null) {
      throw new IllegalStateException("Turn motor has been set twice");
    }
        
    Supplier<Double> speedSupplier;
    Supplier<Rotation2d> angleSupplier;
    Supplier<Rotation2d> angleSpeedSupplier;
    Supplier<Double> distanceSupplier;
    Consumer<Double> speedConsumer;
    Consumer<Rotation2d> angleConsumer;
    double maxSpeedMPS = 0;

    // This uses the turnEncoderAbsolute to reset the motor encoder to make sure they match.
    // The turnEncoderAbsolute times (1/turnEncoderAbsoluteRatio) plus the offset
    // is the angle of the wheel in radians.
    // The normal motor encoder needs to be multiplied by the turnGearRatio then
    // converted from rotations to radians.
    Runnable recalibrate; 

    if (sparkMaxDriveMotor != null) {
      RelativeEncoder encoder = sparkMaxDriveMotor.getEncoder();
      // TODO Check to make sure this is accurate
      encoder.setPositionConversionFactor((wheelDiameter * Math.PI) / (driveGearRatio));
      // TODO Check to make sure this is accurate
      encoder.setVelocityConversionFactor((wheelDiameter * Math.PI) / (driveGearRatio * 60));
      distanceSupplier = encoder::getPosition;
      speedSupplier = encoder::getVelocity;
      SparkPIDController pidController = sparkMaxDriveMotor.getPIDController();
      pidController.setP(driveP, 0);
      pidController.setI(driveI, 0);
      pidController.setD(driveD, 0);
      pidController.setFeedbackDevice(encoder);
      speedConsumer = (Double speed) -> {
        pidController.setReference(speed, ControlType.kVelocity, 0,
            driveFeedforward.calculate(speed));
      };
    } else if (falconDriveMotor != null) {
      // TODO
      distanceSupplier = null;
      speedSupplier = null;
      speedConsumer = null;
      throw new IllegalStateException("Falcons are not supported yet");
    } else {
      throw new IllegalStateException("Drive motor has not been set");
    }

    if (sparkMaxTurnMotor != null) {
      RelativeEncoder encoder = sparkMaxTurnMotor.getEncoder();
      sparkMaxTurnMotor.setInverted(true);
      // TODO Check to make sure this is accurate
      encoder.setPositionConversionFactor((2 * Math.PI) / (turnGearRatio));
      // TODO Check to make sure this is accurate
      encoder.setVelocityConversionFactor((2 * Math.PI) / (turnGearRatio));
      angleSupplier = () -> {
        double angle = encoder.getPosition() % (2 * Math.PI);
        if (angle > Math.PI) {
          angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
          angle += 2 * Math.PI;
        }
        return new Rotation2d(angle);
      };
      angleSpeedSupplier = () -> {
        return new Rotation2d(encoder.getVelocity());
      };
      SparkPIDController pidController = sparkMaxTurnMotor.getPIDController();
      pidController.setP(turnP, 0);
      pidController.setI(turnI, 0);
      pidController.setD(turnD, 0);
      pidController.setFeedbackDevice(encoder);
      angleConsumer = (Rotation2d angle) -> {
        double reference = makeAngleContinuous(encoder.getPosition(), angle.getRadians());
        pidController.setReference(reference, ControlType.kPosition, 0, 
            turnFeedforward.calculate(reference));
      };
      recalibrate = () -> {
        encoder.setPosition(
            makeAngleContinuous(encoder.getPosition(), turnEncoderAbsolute.get())); };
    } else if (falconTurnMotor != null) {
      // TODO
      angleSupplier = null;
      angleConsumer = null;
      angleSpeedSupplier = null;
      throw new IllegalStateException("Falcons are not supported yet");
    } else {
      throw new IllegalStateException("Turn motor has not been set");
    }

    maxSpeedMPS = (driveMotorMaxRPM * wheelDiameter * Math.PI) * 2 / (driveGearRatio * 60) * 4;

    built = true;

    if (location == null || speedSupplier == null || angleSupplier == null
        || distanceSupplier == null || speedConsumer == null
        || angleConsumer == null
        || angleSpeedSupplier == null
        || recalibrate == null || maxSpeedMPS == 0) {
      throw new IllegalStateException("Something went wrong building the SwerveModule");
    }

    Supplier<Double> compensatedDistanceSupplier = () -> {
      return distanceSupplier.get() + (angleSupplier.get().getRotations() * turnToDriveRatio
        * wheelDiameter * Math.PI);
    };

    Supplier<Double> compensatedSpeedSupplier = () -> {
      return speedSupplier.get() + (angleSpeedSupplier.get().getRotations()
        * turnToDriveRatio * wheelDiameter * Math.PI / 60);
    };

    return new SwerveModule(
      location,
      compensatedSpeedSupplier,
      angleSupplier,
      compensatedDistanceSupplier,
      speedConsumer,
      angleConsumer,
      recalibrate,
      maxSpeedMPS,
      sparkMaxDriveMotor != null ? sparkMaxDriveMotor : falconDriveMotor,
      sparkMaxTurnMotor != null ? sparkMaxTurnMotor : falconTurnMotor
    );
  }
}
