package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;

/** Jointly represents the elevator and shooter subsystems. */
public class Superstructure extends SubsystemBase {

  /** Finite state machine for the shooter and elevator. */
  public enum SuperstructureState {
    // TODO - tune presets; also, positives and negatives for indexer might be wrong
    IDLE(0, 3.9835, 0, 0, "Idle"),
    RECEIVE(0, 3.9835, 0.17, 0, "Receive"),
    // AUTO_RECEIVE(0, 3.9835, 0.19, 0, "Auto Receive"),
    AMP_READY(10.2, 4.0966, 0, 0.0, "Amp ready"),
    AMP_GO(10.2, 4.0966, -0.4, 0.0, "Amp go"),
    TRAP_READY(7.4, 0, 0, 0, "Trap ready"),
    TRAP_GO(7.4, 0, 0.2, 0, "Trap go"),
    SPOOLING(7.0, 5.053593, 0, 0.55, "Spooling"),
    SHOOTING(7.0, 5.053593, 0.8, 0.55, "Shooting"),
    PODIUM_READY(7.0, 4.1469, 0, 0.684, "Podium"),
    PODIUM_GO(7.0, 4.1469, 0.8, 0.684, "Podium go"),
    MANUAL_OVERRIDE(0, 0, 0, 0, "Manual override"),
    EJECT_READY(0, 3.9835, 0, 0, "Eject ready"),
    EJECT_FAR_READY(0, 3.9835, 0, 0.35, "Eject far ready"),
    EJECT_GO(0, 3.9835, 0.8, 0.6, "Eject go"),
    VARIABLE_READY(0, 0, 0, 0.9, "Variable ready"),
    VARIABLE_GO(0, 0, 0.8, 0.9, "Variable go");

    public double elevatorEncoderVal;
    public double pivotEncoderVal;
    public double indexerSpeed;
    public double shooterSpeed;
    public String name;

    SuperstructureState(double elevatorEncoderVal, double pivotEncoderVal,
        double indexerSpeed, double shooterSpeed, String name) {
      this.name = name;
      this.elevatorEncoderVal = elevatorEncoderVal;
      this.pivotEncoderVal = pivotEncoderVal;
      this.indexerSpeed = indexerSpeed;
      this.shooterSpeed = shooterSpeed;
    }

    public String toString() {
      return this.name;
    }
  }

  private static SuperstructureState state = SuperstructureState.RECEIVE;

  public static Elevator elevator;
  public static Shooter shooter;
  private static final double TOF_THRESHOLD_MM = 60;

  // Variable state - is there a better way to do this?
  public static double variableElevator = 0;
  public static double variablePivot = 0;
  // public static double variableIndexer = 0;

  /** Constructor. */
  public Superstructure() {
    elevator = new Elevator();
    shooter = new Shooter();
    Superstructure.state = SuperstructureState.IDLE;
    Superstructure.shooter.recalibratePivot();
    // Superstructure.shooter.enable();
  }

  /** Sets state. This might be changed to return if it got rejected
   * because it would have broken the state machine.
   */
  public void setState(SuperstructureState state) {
    Superstructure.state = state;
  }

  public SuperstructureState getState() {
    return state;
  }

  public boolean atSetpoint() {
    return elevator.atSetpoint() && shooter.atSetpoint();
  }

  /** Sub-subsystem for the elevator. */
  public static class Elevator {

    public static CANSparkMax leader;
    private static CANSparkMax follower;
    // private static ElevatorFeedforward feedforward = new ElevatorFeedforward(1.02, 1.39, 2.53);
    // private static ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    /** Constructs a new Elevator object. */
    public Elevator() {
      // TODO - tune max accel, velocity, PID
      // super(new TrapezoidProfile.Constraints(8, 4));
      leader = new CANSparkMax(5, MotorType.kBrushless);
      follower = new CANSparkMax(8, MotorType.kBrushless);
      follower.follow(leader);
      leader.getPIDController().setP(0.3);
      leader.getPIDController().setI(0);
      leader.getPIDController().setD(0);
      leader.getPIDController().setFF(0);
      leader.getPIDController().setOutputRange(-0.275, 1);
      leader.getEncoder().setPositionConversionFactor(1);
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

    public boolean atSetpoint() {
      return state != SuperstructureState.VARIABLE_READY && state != SuperstructureState.VARIABLE_GO
        ? Functions.withinTolerance(leader.getEncoder().getPosition(), state.elevatorEncoderVal, 5)
        : Functions.withinTolerance(leader.getEncoder().getPosition(), variableElevator, 5);
    }

    // public final SysIdRoutine routine = new SysIdRoutine(
        // new SysIdRoutine.Config(
          // Units.Volts.of(3).per(Units.Second),
          // Units.Volts.of(2.5),
          // Units.Seconds.of(5)
        // ),
        // new SysIdRoutine.Mechanism(this::setElevatorVolts, null, this)
    // );
  }

  /** Sub-subsystem for the shooter. */
  public static class Shooter {

    // TODO - tune values and maybe set ShooterFollower to move slower to spin the note slightly
    public static CANSparkMax pivot = new CANSparkMax(13, MotorType.kBrushless);
    public static CANSparkMax indexer;
    private static CANSparkMax shooterLeader; // NOTE: This is a CANSparkFlex
    private static CANSparkMax shooterFollower; // NOTE: This is a CANSparkFlex
    private static final SimpleMotorFeedforward shooterFeedforward
        = new SimpleMotorFeedforward(0, 0.19, 6.90);
    // This might need to be an ArmFeedforward depending on where the CG of the pivot is
    private static final SimpleMotorFeedforward pivotFeedforward
        = new SimpleMotorFeedforward(0, 0, 0); //pivot for shooter
    private static final TimeOfFlight timeOfFlight = new TimeOfFlight(0);
    public static final CANcoder cancoder = new CANcoder(0);
    // Can we get the RPM goal from the PID controller instead of doing this?
    private double rpmGoal = 0;

    public void recalibratePivot() {
      pivot.getEncoder().setPosition(2 * Math.PI * cancoder.getAbsolutePosition().getValueAsDouble());
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
      // super(new TrapezoidProfile.Constraints(210, 90), pivot.getEncoder().getPosition());
      indexer = new CANSparkMax(12, MotorType.kBrushless);
      shooterLeader = new CANSparkMax(52, MotorType.kBrushless);
      shooterFollower = new CANSparkMax(18, MotorType.kBrushless);

      shooterLeader.setInverted(true);
      shooterFollower.setInverted(false);
      shooterFollower.follow(shooterLeader, true);

      // TODO - tune PID
      shooterLeader.getPIDController().setP(0);
      shooterLeader.getPIDController().setI(0);
      shooterLeader.getPIDController().setD(0);
      pivot.getPIDController().setP(0.5);
      pivot.getPIDController().setI(0);
      pivot.getPIDController().setD(0);
      pivot.getEncoder().setPositionConversionFactor(2 * Math.PI / 125);
      pivot.setInverted(true);
      Functions.setStatusFrames(shooterLeader);
      Functions.setStatusFrames(shooterFollower);
      Functions.setStatusFrames(pivot);
      Functions.setStatusFrames(indexer);
      // disable();
      // pivot.getPIDController().setReference(pivot.getEncoder().getPosition(), ControlType.kPosition);
    }

    // @Override
    // protected void useState(TrapezoidProfile.State state) {
      // pivot.getPIDController().setReference(state.position, ControlType.kPosition);
      // 0, pivotFeedforward.calculate(state.velocity));
    // }

    public boolean atSetpoint() {
      return state != SuperstructureState.VARIABLE_READY && state != SuperstructureState.VARIABLE_GO
        ? Functions.withinTolerance(pivot.getEncoder().getPosition(), state.pivotEncoderVal, 5)
        : Functions.withinTolerance(pivot.getEncoder().getPosition(), variablePivot, 5);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.name, null);
    builder.addDoubleProperty("Pivot relative",
        () -> Shooter.pivot.getEncoder().getPosition(), null);
    // builder.addDoubleProperty("ToF", () -> Shooter.timeOfFlight.getRange(), null);
    builder.addDoubleProperty("Elevator encoder",
        () -> Elevator.leader.getEncoder().getPosition(), null);
    builder.addDoubleProperty("Pivot absolute", () -> Shooter.cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI, null);
    // builder.addDoubleProperty("Elevator leader speed controller", Elevator.leader::get, null);
    // builder.addDoubleProperty("Elevator follower speed controller", Elevator.follower::get, null);
    // builder.addDoubleProperty("Elevator leader encoder vel",
        // Elevator.leader.getEncoder()::getVelocity, null);
    // builder.addDoubleProperty("Elevator follower encoder vel",
        // Elevator.follower.getEncoder()::getVelocity, null);
    builder.addDoubleProperty("Elevator leader temperature",
        Elevator.leader::getMotorTemperature, null);
    builder.addDoubleProperty("Elevator follower temperature",
        Elevator.follower::getMotorTemperature, null);
    // builder.addDoubleProperty("Elevator lead current", Elevator.leader::getOutputCurrent, null);
    // builder.addDoubleProperty("Elevator follow current", Elevator.follower::getOutputCurrent, null);
    // builder.addDoubleProperty("Elevator voltage leader", Elevator.follower::getBusVoltage, null);
    // builder.addDoubleProperty("Elevator applied output", Elevator.leader::getAppliedOutput, null);
    // builder.addBooleanProperty("At setpoint", this::atSetpoint, null);
  }
}
