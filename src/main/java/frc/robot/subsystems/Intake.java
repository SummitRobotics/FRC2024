package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents the intake subsystem. */
public class Intake extends SubsystemBase {

  /** Finite state machine options for the intake. */
  public enum State {
    UP,
    DOWN,
    OPENING,
    CLOSING
  }
  
  private static State state;
  // TODO - set CAN IDs
  private static final CANSparkMax pivot = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax roller = new CANSparkMax(0, MotorType.kBrushless);
  private static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  /** Constructs a new Intake object. */
  public Intake() {
    pivot.getPIDController().setP(0.004);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
  }

  public State getState() {
    return state;
  }

  public void setState(State state) {
    Intake.state = state;
  }

  @Override
  public void periodic() {
    switch (state) {
      case UP:
      case DOWN:
      case OPENING:
      case CLOSING:
      default:
    }
  }
}
