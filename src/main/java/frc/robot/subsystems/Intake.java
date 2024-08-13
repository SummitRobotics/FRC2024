package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.Functions;

/** Represents the intake subsystem. */
public class Intake extends SubsystemBase {

  /** Finite state machine options for the intake. */
  public enum IntakeState {
    UP(58, 0, "Up"),
    DOWN(0, 0.5, "Down"),
    MID(23, 0, "Mid"),
    MANUAL_OVERRIDE(0, 0, "Manual override");

    // The whole TriggerForState thing uses functional programming to automatically generate public triggers for each state.
    // These public triggers are readable from elsewhere with something like `getStateTrigger(IntakeState.DOWN)`; see getStateTrigger() below.
    // This avoids having to do `new Trigger(intake.getState == IntakeState.DOWN)` at every call site, which might be preferable anyways.
    // Alternatively, it gets around writing `public Trigger up() { return new Trigger(() -> state == IntakeState.UP); }` for every state in this file.

    private interface TriggerForState {
      public Trigger get(IntakeState state);
    };

    private double pivot;
    private double roller;
    private String name;
    private TriggerForState triggerGetter;
    private IntakeState(double pivot, double roller, String name) {
      this.name = name;
      this.pivot = pivot;
      this.roller = roller;
      triggerGetter = (IntakeState other) -> new Trigger(() -> other == this);
    }

    public String toString() {
      return this.name;
    }
  }

  private static IntakeState state;
  private static CANSparkMax pivot;
  private static CANSparkMax roller;
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

  // Generates a public trigger for checking if the intake is in that state.
  public Trigger getStateTrigger(IntakeState state) {
    return state.triggerGetter.get(state);
  }

  private void setRoller(double val) {
    roller.set(val);
  }

  private void setPivot(double val) {
    pivot.set(val);
  }

  /** Whether or not pivot is at setpoint. */
  public Trigger atSetpoint() {
    return new Trigger(() -> Functions.withinTolerance(pivot.getEncoder().getPosition(), state.pivot, 2));
  }

  private void setPivotReference(double encoderRotations) {
    pivot.getPIDController().setReference(encoderRotations, ControlType.kPosition, 0,
      feedforward.calculate(encoderRotations * 2 * Math.PI / 214, 0)); // For gear reduction
  }

  public Command setState(IntakeState state) {
    return this.runOnce(() -> Intake.state = state);
  }

  public Command togglePivot() {
    return this.runOnce(() -> state = state == IntakeState.DOWN ? IntakeState.UP : IntakeState.DOWN);
  }

  public Command toggleMO() {
    return this.runOnce(() -> state = state == IntakeState.MANUAL_OVERRIDE ? IntakeState.DOWN : IntakeState.MANUAL_OVERRIDE);
  }

  public Command manualOperateCommand(double pivotVal, double rollerVal) {
    return this.runOnce(() -> {
      if (state == IntakeState.MANUAL_OVERRIDE) {
        setRoller(rollerVal);
        setPivot(pivotVal);
      }
    });
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous()) state = IntakeState.DOWN;
    if (state != IntakeState.MANUAL_OVERRIDE) {
      setRoller(state.roller);
      setPivotReference(state.pivot);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.name, null);
    builder.addDoubleProperty("Pivot encoder", () -> pivot.getEncoder().getPosition(), null);
    builder.addBooleanProperty("At setpoint", atSetpoint()::getAsBoolean, null);
  }
}
