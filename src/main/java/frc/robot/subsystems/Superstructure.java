package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.GoodTrapezoidProfileSubsystem;

/** Jointly represents the elevator and shooter subsystems. */
public class Superstructure extends SubsystemBase {

  /** Finite state machine for the shooter and elevator. */
  public enum SuperstructureState {
    // TODO - tune presets; also, positives and negatives for indexer might be wrong
    IDLE(0, 0, 0),
    RECEIVE(0, 0, -0.2),
    AMP_READY(7.4, 0, 0),
    AMP_GO(7.4, 0, 0.2),
    TRAP_READY(7.4, 0, 0),
    TRAP_GO(7.4, 0, 0.2),
    SPOOLING(7.4, 0, 0),
    SHOOTING(7.4, 0, -0.2),
    MANUAL_OVERRIDE(0, 0, 0);

    public double elevatorEncoderVal;
    public double pivotEncoderVal;
    public double indexerSpeed;
    SuperstructureState(double elevatorEncoderVal, double pivotEncoderVal, double indexerSpeed) {
      this.elevatorEncoderVal = elevatorEncoderVal;
      this.pivotEncoderVal = pivotEncoderVal;
      this.indexerSpeed = indexerSpeed;
    }

    public String toString() {
      if (this == SuperstructureState.IDLE) return "Idle";
      if (this == SuperstructureState.RECEIVE) return "Receive";
      if (this == SuperstructureState.AMP_READY) return "Amp ready";
      if (this == SuperstructureState.AMP_GO) return "Amp go";
      if (this == SuperstructureState.TRAP_READY) return "Trap ready";
      if (this == SuperstructureState.TRAP_GO) return "Trap go";
      if (this == SuperstructureState.SPOOLING) return "Spooling";
      if (this == SuperstructureState.SHOOTING) return "Shooting";
      if (this == SuperstructureState.MANUAL_OVERRIDE) return "Manual Override";

      return "";
    }
  }

  private static SuperstructureState state = SuperstructureState.IDLE;

  public static Elevator elevator = new Elevator();
  public static Shooter shooter = new Shooter();
  // TODO - set
  private static final double TOF_THRESHOLD_MM = 75;

  public Superstructure() {
    Superstructure.state = SuperstructureState.RECEIVE;
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
  public static class Elevator extends GoodTrapezoidProfileSubsystem {

    // TODO - IDs
    public static CANSparkMax leader = new CANSparkMax(5, MotorType.kBrushless);
    public static CANSparkMax follower = new CANSparkMax(8, MotorType.kBrushless);
    // private static ElevatorFeedforward feedforward = new ElevatorFeedforward(0.62, 4.39, 1.53);
    private static ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    /** Constructs a new Elevator object. */
    public Elevator() {
      // TODO - tune max accel, velocity, PID
      super(new TrapezoidProfile.Constraints(35, 15));
      // follower.follow(leader);
      leader.getPIDController().setP(0.004);
      leader.getPIDController().setI(0);
      leader.getPIDController().setD(0);
      follower.getPIDController().setP(leader.getPIDController().getP());
      follower.getPIDController().setI(leader.getPIDController().getI());
      follower.getPIDController().setD(leader.getPIDController().getD());
    }

    public void setElevator(double val) {
      leader.set(val);
      follower.set(val);
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
      leader.getPIDController().setReference(setpoint.position, ControlType.kPosition//, 0,
          /*feedforward.calculate(setpoint.velocity)*/);
      follower.getPIDController().setReference(setpoint.position, ControlType.kPosition//, 0,
          /*feedforward.calculate(setpoint.velocity)*/);
    }
  }

  /** Sub-subsystem for the shooter. */
  public static class Shooter extends GoodTrapezoidProfileSubsystem {

    // TODO - tune values and maybe set ShooterFollower to move slower to spin the note slightly
    public static final CANSparkMax pivot = new CANSparkMax(13, MotorType.kBrushless);
    public static final CANSparkMax indexer = new CANSparkMax(10, MotorType.kBrushless);
    private static final CANSparkMax shooterLeader = new CANSparkMax(12, MotorType.kBrushless);
    private static final CANSparkMax shooterFollower = new CANSparkMax(16, MotorType.kBrushless);
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
      super(new TrapezoidProfile.Constraints(0, 0));
      shooterFollower.setInverted(true);
      // shooterFollower.follow(shooterLeader);
      // TODO - tune PID
      shooterLeader.getPIDController().setP(0.000);
      shooterLeader.getPIDController().setI(0);
      shooterLeader.getPIDController().setD(0);
      shooterFollower.getPIDController().setP(shooterLeader.getPIDController().getP());
      shooterFollower.getPIDController().setI(shooterLeader.getPIDController().getI());
      shooterFollower.getPIDController().setD(shooterLeader.getPIDController().getD());
      pivot.getPIDController().setP(0.004);
      pivot.getPIDController().setI(0);
      pivot.getPIDController().setD(0);
    }

    @Override
    protected void useState(TrapezoidProfile.State state) {
      pivot.getPIDController().setReference(state.position, ControlType.kPosition,
          0, pivotFeedforward.calculate(state.velocity));
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
  }
}
