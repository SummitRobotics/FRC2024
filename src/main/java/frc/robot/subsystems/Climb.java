package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.GoodTrapezoidProfileSubsystem;

/** Represents the climb subsystem. */
public class Climb extends SubsystemBase {

  // TODO - tune
  public Arm armLeft = new Arm(0);
  public Arm armRight = new Arm(0);
  public static double HEIGHT = 0;
  public static double CURRENT = 0;

  /** Subclass representing the motor and feedforward for an individual arm. */
  public class Arm extends GoodTrapezoidProfileSubsystem {
    CANSparkMax motor;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

    public Arm(int id) {
      super(new TrapezoidProfile.Constraints(0, 0));
      motor = new CANSparkMax(id, MotorType.kBrushless);
    }

    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
      motor.getPIDController().setReference(setpoint.position,
          ControlType.kPosition, 0, feedforward.calculate(setpoint.velocity));
    }
  }
}
