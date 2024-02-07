package frc.robot.utilities;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Cpoy of WPILib's stock TrapezoidProfileSubsystem, except you can tell if the goal is reached.
 * Is there a better way to do this?
 */
public abstract class GoodTrapezoidProfileSubsystem extends SubsystemBase {
  private final double m_period;
  private final TrapezoidProfile m_profile;

  private TrapezoidProfile.State m_state;
  private TrapezoidProfile.State m_goal;

  private boolean m_enabled = true;

  /**
   * Creates a new TrapezoidProfileSubsystem.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   * @param initialPosition The initial position of the controlled mechanism when the subsystem is
   *     constructed.
   * @param period The period of the main robot loop, in seconds.
   */
  public GoodTrapezoidProfileSubsystem(
      TrapezoidProfile.Constraints constraints, double initialPosition, double period) {
    requireNonNullParam(constraints, "constraints", "TrapezoidProfileSubsystem");
    m_profile = new TrapezoidProfile(constraints);
    m_state = new TrapezoidProfile.State(initialPosition, 0);
    setGoal(initialPosition);
    m_period = period;
  }

  /**
   * Creates a new TrapezoidProfileSubsystem.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   * @param initialPosition The initial position of the controlled mechanism when the subsystem is
   *     constructed.
   */
  public GoodTrapezoidProfileSubsystem(
      TrapezoidProfile.Constraints constraints, double initialPosition) {
    this(constraints, initialPosition, 0.02);
  }

  /**
   * Creates a new TrapezoidProfileSubsystem.
   *
   * @param constraints The constraints (maximum velocity and acceleration) for the profiles.
   */
  public GoodTrapezoidProfileSubsystem(TrapezoidProfile.Constraints constraints) {
    this(constraints, 0, 0.02);
  }

  @Override
  public void periodic() {
    m_state = m_profile.calculate(m_period, m_goal, m_state);
    if (m_enabled) {
      useState(m_state);
    }
  }

  /**
   * Sets the goal state for the subsystem.
   *
   * @param goal The goal state for the subsystem's motion profile.
   */
  public final void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
  }

  /**
   * Sets the goal state for the subsystem. Goal velocity assumed to be zero.
   *
   * @param goal The goal position for the subsystem's motion profile.
   */
  public final void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
  }

  public TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  /** Enable the TrapezoidProfileSubsystem's output. */
  public void enable() {
    m_enabled = true;
  }

  /** Disable the TrapezoidProfileSubsystem's output. */
  public void disable() {
    m_enabled = false;
  }

  /** Returns if the profile is near the setpoint. */
  public boolean atSetpoint() {
    // TODO - tune tolerances
    return Functions.withinTolerance(m_state.position, m_goal.position, 2)
        && Functions.withinTolerance(m_state.velocity, 0, 2);
  }

  /**
   * Users should override this to consume the current state of the motion profile.
   *
   * @param state The current state of the motion profile.
   */
  protected abstract void useState(TrapezoidProfile.State state);
}