package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utilities.Functions;

/** Represents the intake subsystem. */
public class Intake extends SubsystemBase {

  // TODO - use SmartMotion profiling instead of WPILib

  /** Finite state machine options for the intake. */
  public enum IntakeState {
    UP(58, 0, "Up"),
    DOWN(0, 0.5, "Down"),
    MID(23, 0, "Mid"),
    MANUAL_OVERRIDE(0, 0, "Manual override");

    public double pivot;
    public double roller;
    public String name;
    private IntakeState(double pivot, double roller, String name) {
      this.name = name;
      this.pivot = pivot;
      this.roller = roller;
    }

    public String toString() {
      return this.name;
    }
  }

  private static IntakeState state;
  public static SuperstructureState pivotUpandDown;
  public static CANSparkMax pivot;
  public static CANSparkMax roller;
  private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0.21, 4.17);

  /** Constructs a new Intake object. */
  public Intake() {
    // super(new TrapezoidProfile.Constraints(500, 200));
    pivot = new CANSparkMax(6, MotorType.kBrushless);
    pivot.restoreFactoryDefaults();
    roller = new CANSparkMax(7, MotorType.kBrushless);
    Functions.setStatusFrames(pivot);
    Functions.setStatusFrames(roller);
    pivot.getPIDController().setP(0.48);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0.30);
    state = IntakeState.UP;
    pivot.setSmartCurrentLimit(40);
    roller.setSmartCurrentLimit(30);
    pivot.getEncoder().setPosition(58);
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

  public void setReference(double encoderRotations) {
    pivot.getPIDController().setReference(encoderRotations, ControlType.kPosition, 0,
      feedforward.calculate(encoderRotations * 2 * Math.PI / 214, 0)); // For gear reduction
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.name, null);
    builder.addDoubleProperty("Pivot encoder", () -> pivot.getEncoder().getPosition(), null);
    builder.addBooleanProperty("At setpoint", this::atSetpoint, null);
  }
}
