package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utilities.ConfigManager;
import frc.robot.utilities.Functions;

/** Represents the intake subsystem. */
public class Intake extends TrapezoidProfileSubsystem {

  /** Finite state machine options for the intake. */
  public enum IntakeState {
    UP(0, 0),
    DOWN(-58, 0.5),
    MID(-29, 0),
    MANUAL_OVERRIDE(0, 0);

    public String toString() {
      if (this == UP) return "Up";
      if (this == DOWN) return "Down";
      if (this == MID) return "Mid";
      return "Manual Override";
    }

    public double pivot;
    public double roller;
    private IntakeState(double pivot, double roller) {
      this.pivot = pivot;
      this.roller = roller;
    }
  }

  private static final ConfigManager.PrefixedConfigAccessor config = ConfigManager.getInstance().getPrefixedAccessor("Intake.");
  private static IntakeState state;
  public static SuperstructureState pivotUpandDown;
  public static CANSparkMax pivot;
  public static CANSparkMax roller;
  // private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 4.38);
  public static final double DOWNPOSITION = config.getDouble("downPosition", -58);

  /** Constructs a new Intake object. */
  public Intake() {
    super(new TrapezoidProfile.Constraints(
      config.getDouble("maxVelocity", 60),
      config.getDouble("maxAcceleration", 25)
    ));
    pivot = new CANSparkMax(6, MotorType.kBrushless);
    roller = new CANSparkMax(7, MotorType.kBrushless);
    Functions.setStatusFrames(pivot);
    Functions.setStatusFrames(roller);
    pivot.getPIDController().setP(config.getDouble("pivot.PID.P", 0.02));
    pivot.getPIDController().setI(config.getDouble("pivot.PID.I", 0));
    pivot.getPIDController().setD(config.getDouble("pivot.PID.D", 0.005));
    state = IntakeState.UP;
  }

  public IntakeState getState() {
    return state;
  }

  public void setState(IntakeState state) {
    Intake.state = state;
  }

  public void setRoller(double val) {
    roller.set(val);
  }

  public void setPivot(double val) {
    pivot.set(val);
  }

  /** Whether or not pivot is at setpoint. */
  public boolean atSetpoint() {
    return Functions.withinTolerance(pivot.getEncoder().getPosition(), state.pivot, 2);
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    pivot.getPIDController().setReference(setpoint.position,
        ControlType.kPosition/*, 0, feedforward.calculate(setpoint.position, setpoint.velocity)*/);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.toString(), null);
    builder.addDoubleProperty("Pivot encoder", () -> pivot.getEncoder().getPosition(), null);
    builder.addBooleanProperty("At setpoint", this::atSetpoint, null);
  }
}
