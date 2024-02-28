package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Functions;
import frc.robot.utilities.GoodTrapezoidProfileSubsystem;

/** Represents the climb subsystem. */
public class Climb extends SubsystemBase {

  // TODO - tune
  public Arm armLeft;
  public Arm armRight;
  public static double HEIGHT = 0;
  public static double OFF_GROUND = 0;
  public static double CURRENT = 0;

  public Climb() {
    armLeft = new Arm(14);
    armRight = new Arm(15);
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
  
  public boolean atSetpoint() {
    return armLeft.atSetpoint() && armRight.atSetpoint();
  }

  /** Subclass representing the motor and feedforward for an individual arm. */
  public class Arm extends GoodTrapezoidProfileSubsystem {
    public CANSparkMax motor;
    // private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    /** Constructor. */
    public Arm(int id) {
      super(new TrapezoidProfile.Constraints(25, 12));
      motor = new CANSparkMax(id, MotorType.kBrushless);
      Functions.setStatusFrames(motor);
      motor.setSoftLimit(SoftLimitDirection.kForward, 0);
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
    builder.addDoubleProperty("Climb encoder",
        () -> armLeft.motor.getEncoder().getPosition(), null);
    // builder.addDoubleProperty("Climb current",
        // () -> armLeft.motor.getOutputCurrent(), null);
  }
}
