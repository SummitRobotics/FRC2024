package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

/** Represents the elevator. Uses T R A P E Z O I D S. */
public class Elevator extends TrapezoidProfileSubsystem {

  /** Finite state machine options. */
  public enum ElevatorState {
    // TODO - tune encoder values
    RECEIVE(0),
    AMP(0),
    TRAP(0),
    SPEAKER(0),
    CLIMB(0);

    public double encoderVal;
    ElevatorState(double encoderVal) {
      this.encoderVal = encoderVal;
    }
  }
 
  private static ElevatorState state;

  // TODO - tune values
  private static final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax follower = new CANSparkMax(0, MotorType.kBrushless);
  private static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  /** Constructs a new elevator object. */
  public Elevator() {
    // TODO - tune max velocity, accel, and PID
    super(new TrapezoidProfile.Constraints(0, 0));
    follower.follow(leader);
    leader.getPIDController().setP(0.04);
    leader.getPIDController().setI(0);
    leader.getPIDController().setD(0);
  }

  public static ElevatorState getState() {
    return state;
  }

  public static void setState(ElevatorState state) {
    Elevator.state = state;
  }

  @Override
  public void periodic() {
    setGoal(state.encoderVal);
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    leader.getPIDController().setReference(setpoint.position, ControlType.kPosition, 0,
        feedforward.calculate(setpoint.velocity));
  }
}
