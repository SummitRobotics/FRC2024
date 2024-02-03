package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utilities.GoodTrapezoidProfileSubsystem;

/** Represents the intake subsystem. */
public class Intake extends GoodTrapezoidProfileSubsystem {

  /** Finite state machine options for the intake. */
  public enum IntakeState {
    UP,
    DOWN,
    MANUAL_OVERRIDE;
  }
  
  private static IntakeState state;
  public static SuperstructureState pivotUpandDown;
  // TODO - set CAN IDs
  private static final CANSparkMax pivot = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax roller = new CANSparkMax(0, MotorType.kBrushless);
  // private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);
  public static final double DOWNPOSITION = 0;

  /** Constructs a new Intake object. */
  public Intake() {
    super(new TrapezoidProfile.Constraints(0, 0));
    pivot.getPIDController().setP(0.004);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
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

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    // TODO - figure out units for feedforward
    pivot.getPIDController().setReference(setpoint.position, ControlType.kPosition/*, 0, feedforward.calculate(setpoint.position, setpoint.velocity)*/);
  }  
}
