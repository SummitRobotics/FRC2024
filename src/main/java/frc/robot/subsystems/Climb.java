package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.utilities.Functions;

/** Represents the climb subsystem. */
public class Climb extends SubsystemBase {

  // TODO - tune
  public Arm armLeft;
  public Arm armRight;

  public enum ClimbState {
    MANUAL_OVERRIDE,
    IDLE,
    AUTO;
  }

  private ClimbState state = ClimbState.IDLE;
  public static final double ABOVE_CHAIN = 0;
  public static final double DOWN = 0;
  public static final double BELOW_CHAIN = 0;
  public static double CURRENT = 0;

  public Climb() {
    armLeft = new Arm(14);
    armRight = new Arm(15);
  }

  public ClimbState getState() {
    return state;
  }

  public void setState(ClimbState state) {
    this.state = state;
  }

  public void setGoal(double setpoint) {
    armLeft.setGoal(setpoint);
    armRight.setGoal(setpoint);
  }

  public void set(double val) {
    armLeft.motor.set(val);
    armRight.motor.set(val);
  }

  public void setLeft(double val) {
    armLeft.motor.set(val);
  }

  public void setRight(double val) {
    armRight.motor.set(val);
  }
  
  /** Subclass representing the motor and feedforward for an individual arm. */
  public class Arm extends TrapezoidProfileSubsystem {
    public CANSparkMax motor;
    // private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    /** Constructor. */
    public Arm(int id) {
      super(new TrapezoidProfile.Constraints(25, 12));
      motor = new CANSparkMax(id, MotorType.kBrushless);
      Functions.setStatusFrames(motor);
      motor.setSoftLimit(SoftLimitDirection.kReverse, 0.1f);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      motor.setSoftLimit(SoftLimitDirection.kForward, 130f);
      motor.enableSoftLimit(SoftLimitDirection.kForward, false);
      disable();
      motor.getEncoder().setPosition(0);
    }

    public boolean getCurrent() {
      return motor.getOutputCurrent() >= CURRENT;
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
      motor.getPIDController().setReference(setpoint.position,
          ControlType.kPosition, 0/*, feedforward.calculate(setpoint.velocity)*/);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Climb left encoder",
        () -> armLeft.motor.getEncoder().getPosition(), null);
      builder.addDoubleProperty("Climb right motor", () -> armRight.motor.getEncoder().getPosition(), null);
    // builder.addDoubleProperty("Climb current",
        // () -> armLeft.motor.getOutputCurrent(), null);
  }
}
