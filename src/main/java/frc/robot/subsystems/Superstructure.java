package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utilities.Functions;
import frc.robot.utilities.ConfigManager;

/** Jointly represents the elevator and shooter subsystems. */
public class Superstructure extends SubsystemBase {

  /** Finite state machine for the shooter and elevator. */
  public enum SuperstructureState {
    // TODO - tune presets; also, positives and negatives for indexer might be wrong
    IDLE(0, -0.373, 0, 0),
    RECEIVE(0, -0.373, 0.17, 0),
    AMP_READY(3.3, 20, 0, 0.0),
    AMP_GO(3.3, 20, -0.45, 0.0),
    TRAP_READY(7.4, 0, 0, 0),
    TRAP_GO(7.4, 0, 0.2, 0),
    SPOOLING(7.0, -11, 0, 0.8),
    SHOOTING(7.0, -11, 0.4, 0.8),
    MANUAL_OVERRIDE(0, 0, 0, 0);

    public String name;
    public double elevatorEncoderVal;
    public double pivotEncoderVal;
    public double indexerSpeed;
    public double shooterSpeed;

    SuperstructureState(double elevatorEncoderVal, double pivotEncoderVal, double indexerSpeed, double shooterSpeed) {
      this.elevatorEncoderVal = elevatorEncoderVal;
      this.pivotEncoderVal = pivotEncoderVal;
      this.indexerSpeed = indexerSpeed;
      this.shooterSpeed = shooterSpeed;
    }

    public String toString() {
      if (this == SuperstructureState.IDLE) {
        return "Idle";
      }
      if (this == SuperstructureState.RECEIVE) {
        return "Receive";
      }
      if (this == SuperstructureState.AMP_READY) {
        return "Amp ready";
      }
      if (this == SuperstructureState.AMP_GO) {
        return "Amp go";
      }
      if (this == SuperstructureState.TRAP_READY) {
        return "Trap ready";
      }
      if (this == SuperstructureState.TRAP_GO) {
        return "Trap go";
      }
      if (this == SuperstructureState.SPOOLING) {
        return "Spooling";
      }
      if (this == SuperstructureState.SHOOTING) {
        return "Shooting";
      }
      if (this == SuperstructureState.MANUAL_OVERRIDE) {
        return "Manual Override";
      }

      return "";
    }
  }

  private static final ConfigManager.PrefixedConfigAccessor config = ConfigManager.getInstance().getPrefixedAccessor("SuperStructure.");
  private static SuperstructureState state = SuperstructureState.IDLE;

  public static Elevator elevator;
  public static Shooter shooter;
  // TODO - set
  private static final double TOF_THRESHOLD_MM = config.getDouble("tof_threshold_mm", 60);
  
  /** Constructor. */
  public Superstructure() {
    elevator = new Elevator();
    shooter = new Shooter();
    Superstructure.state = SuperstructureState.IDLE;
    Superstructure.shooter.recalibratePivot();
  }

  /** Sets state. This might be changed to return if it got rejected
   * because it would have broken the state machine.
   */
  public void setState(SuperstructureState state) {
    // if (Superstructure.state == SuperstructureState.IDLE
    // || (Superstructure.state == SuperstructureState.AMP_READY
    // && state == SuperstructureState.AMP_GO)
    // || (Superstructure.state == SuperstructureState.TRAP_READY
    // && state == SuperstructureState.TRAP_GO)
    // || (Superstructure.state == SuperstructureState.SPOOLING
    // && state == SuperstructureState.SHOOTING)) {
    // Superstructure.state = state;
    // return true;
    // } else {
    // System.out.println("Shooter / elevator rejected illegal state change");
    // return false;
    // }
    Superstructure.state = state;
  }

  public SuperstructureState getState() {
    return state;
  }

  public boolean atSetpoint() {
    return elevator.atSetpoint() && shooter.atSetpoint();
    // return shooter.atSetpoint();
  }

  // This should probably all happen in a command instead of periodic()
  // @Override
  // public void periodic() {
  // This covers all behavior besides state transitions
  // and stuff for SPOOLING, SHOOTING, and MANUAL_OVERRIDE
  // if (state != SuperstructureState.MANUAL_OVERRIDE) {
  // elevator.setGoal(state.elevatorEncoderVal);
  // shooter.setGoal(state.pivotEncoderVal);
  // Shooter.indexer.set(state.indexerSpeed);
  // }

  // switch (state) {
  // case RECEIVE:
  // if (Shooter.timeOfFlight.getRange() >= TOF_THRESHOLD_MM) {
  // state = SuperstructureState.IDLE;
  // }
  // break;
  // default:
  // }
  // }

  /** Sub-subsystem for the elevator. */
  public static class Elevator extends TrapezoidProfileSubsystem {

    private static final ConfigManager.PrefixedConfigAccessor config = ConfigManager.getInstance().getPrefixedAccessor("Elevator.");
    public static CANSparkMax leader;
    private static CANSparkMax follower;
    // private static ElevatorFeedforward feedforward = new ElevatorFeedforward(1.02, 1.39, 2.53);
    // private static ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    /** Constructs a new Elevator object. */
    public Elevator() {
      // TODO - tune max accel, velocity, PID
      super(new TrapezoidProfile.Constraints(
        config.getDouble("maxVelocity", 8),
        config.getDouble("maxAcceleration", 4)
      ));
      leader = new CANSparkMax(5, MotorType.kBrushless);
      follower = new CANSparkMax(8, MotorType.kBrushless);
      follower.follow(leader);

      leader.getPIDController().setP(config.getDouble("PID.P", 0.3));
      leader.getPIDController().setI(config.getDouble("PID.I", 0));
      leader.getPIDController().setD(config.getDouble("PID.D", 0));
      leader.getPIDController().setFF(config.getDouble("PID.FF", 0));
      leader.getPIDController().setOutputRange(
        config.getDouble("PID.OutputRange.Min", -0.2),
        config.getDouble("PID.OutputRange.Max", 1)
      );

      leader.getEncoder().setPositionConversionFactor(1 / 125);
      Functions.setStatusFrames(leader);
      Functions.setStatusFrames(follower);
    }

    public void setElevator(double val) {
      leader.set(val);
      // follower.set(val);
    }

    public void setElevatorVolts(Measure<Voltage> volts) {
      leader.setVoltage(volts.in(Units.Volts));
      follower.setVoltage(volts.in(Units.Volts));
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {

      // leader.getPIDController().setReference(setpoint.position, ControlType.kPosition); //0,
          // feedforward.calculate(setpoint.velocity));
      // follower.getPIDController().setReference(setpoint.position, ControlType.kPosition//, 0,
          // /*feedforward.calculate(setpoint.velocity)*/);
    }

    public boolean atSetpoint() {
      return Functions.withinTolerance(leader.getEncoder().getPosition(),
          state.elevatorEncoderVal, 0.5);
    }

    public final SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
          Units.Volts.of(3).per(Units.Second),
          Units.Volts.of(2.5),
          Units.Seconds.of(5)
        ),
        new SysIdRoutine.Mechanism(this::setElevatorVolts, null, this)
    );
  }

  /** Sub-subsystem for the shooter. */
  public static class Shooter extends TrapezoidProfileSubsystem {

    private static final ConfigManager.PrefixedConfigAccessor config = ConfigManager.getInstance().getPrefixedAccessor("Shooter.");

    // TODO - tune values and maybe set ShooterFollower to move slower to spin the note slightly
    public static CANSparkMax pivot;
    public static CANSparkMax indexer;
    private static CANSparkMax shooterLeader; // NOTE: This is a CANSparkFlex
    private static CANSparkMax shooterFollower; // NOTE: This is a CANSparkFlex
    private static final SimpleMotorFeedforward shooterFeedforward
        = new SimpleMotorFeedforward(0, 0.19, 6.90);
    // This might need to be an ArmFeedforward depending on where the CG of the pivot is
    private static final SimpleMotorFeedforward pivotFeedforward
        = new SimpleMotorFeedforward(0, 0, 0); //pivot for shooter
    private static final TimeOfFlight timeOfFlight = new TimeOfFlight(0);
    private static final CANcoder cancoder = new CANcoder(0);
    // Can we get the RPM goal from the PID controller instead of doing this?
    private double rpmGoal = 0;

    public void recalibratePivot() {
      pivot.getEncoder().setPosition(cancoder.getAbsolutePosition().getValueAsDouble());
    }

    public boolean getToF() {
      return timeOfFlight.getRange() <= TOF_THRESHOLD_MM;
    }

    public void setIndexer(double val) {
      indexer.set(val);
    }

    /** Sets target RPM of shooter.
     * Should be called setShooterRPM but checkstyle doesn't like that
     *
     * @param val target velocity in RPM
     */
    public void setShooterRpm(double val) {
      rpmGoal = val;
      shooterLeader.getPIDController().setReference(val, ControlType.kVelocity, 0,
          shooterFeedforward.calculate(val));
      // TODO - setting as follower should work better for this
      shooterFollower.getPIDController().setReference(val, ControlType.kVelocity, 0,
          shooterFeedforward.calculate(val));
    }

    public boolean isSpooled() {
      // TODO - tune tolerance
      return Functions.withinTolerance(shooterLeader.getEncoder().getVelocity(), rpmGoal, 5);
    }

    public void setShooter(double val) {
      shooterLeader.set(val);
      shooterFollower.set(val);
    }

    /** Creates a new Shooter object. */
    public Shooter() {
      super(new TrapezoidProfile.Constraints(
        config.getDouble("maxVelocity", 35),
        config.getDouble("maxAcceleration", 15)
      ));
      pivot = new CANSparkMax(13, MotorType.kBrushless);
      indexer = new CANSparkMax(12, MotorType.kBrushless);
      shooterLeader = new CANSparkMax(52, MotorType.kBrushless);
      shooterFollower = new CANSparkMax(18, MotorType.kBrushless);

      shooterLeader.setInverted(true);
      shooterFollower.setInverted(false);
      shooterFollower.follow(shooterLeader, true);

      // TODO - tune PID
      shooterLeader.getPIDController().setP(config.getDouble("PID.P", 0));
      shooterLeader.getPIDController().setI(config.getDouble("PID.I", 0));
      shooterLeader.getPIDController().setD(config.getDouble("PID.D", 0));
      pivot.getPIDController().setP(config.getDouble("Pivot.PID.P", 0.05));
      pivot.getPIDController().setI(config.getDouble("Pivot.PID.I", 0));
      pivot.getPIDController().setD(config.getDouble("Pivot.PID.D", 0));
      Functions.setStatusFrames(shooterLeader);
      Functions.setStatusFrames(shooterFollower);
      Functions.setStatusFrames(pivot);
      Functions.setStatusFrames(indexer);
    }

    @Override
    protected void useState(TrapezoidProfile.State state) {
      pivot.getPIDController().setReference(state.position, ControlType.kPosition);
      // 0, pivotFeedforward.calculate(state.velocity));
    }

    public boolean atSetpoint() {
      return Functions.withinTolerance(pivot.getEncoder().getPosition(), state.pivotEncoderVal, 5);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.toString(), null);
    builder.addDoubleProperty("Pivot encoder",
        () -> Shooter.pivot.getEncoder().getPosition(), null);
    builder.addDoubleProperty("ToF", () -> Shooter.timeOfFlight.getRange(), null);
    builder.addDoubleProperty("Elevator encoder",
        () -> Elevator.leader.getEncoder().getPosition(), null);
    builder.addDoubleProperty("Elevator leader speed controller", Elevator.leader::get, null);
    builder.addDoubleProperty("Elevator follower speed controller", Elevator.follower::get, null);
    builder.addDoubleProperty("Elevator leader encoder vel",
        Elevator.leader.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Elevator follower encoder vel",
        Elevator.follower.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Elevator leader temperature",
        Elevator.leader::getMotorTemperature, null);
    builder.addDoubleProperty("Elevator follower temperature",
        Elevator.follower::getMotorTemperature, null);
    builder.addDoubleProperty("Elevator lead current", Elevator.leader::getOutputCurrent, null);
    builder.addDoubleProperty("Elevator follow current", Elevator.follower::getOutputCurrent, null);
    builder.addDoubleProperty("Elevator voltage leader", Elevator.follower::getBusVoltage, null);
    builder.addDoubleProperty("Elevator applied output", Elevator.leader::getAppliedOutput, null);
    builder.addBooleanProperty("At setpoint", this::atSetpoint, null);
  }
}
