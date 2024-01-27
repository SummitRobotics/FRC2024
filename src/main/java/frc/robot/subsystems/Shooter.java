package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents the shooter subsystem. */
public class Shooter extends SubsystemBase {
  
  /** Finite state machine for the shooter. */
  public enum ShooterState {
    SPOOLING(0),
    SHOOTING(0),
    // AMP(0),
    // TRAP(0),
    RECEIVE(0),
    IDLE(0);

    public double pivotEncoderVal;
    ShooterState(double pivotEncoderVal) {
      this.pivotEncoderVal = pivotEncoderVal;
    }
  }

  private static ShooterState state;
  // TODO - tune
  private static final CANSparkMax pivot = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax indexer = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax shooterLeader = new CANSparkMax(0, MotorType.kBrushless);
  private static final CANSparkMax shooterFollower = new CANSparkMax(0, MotorType.kBrushless);
  private static final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(0, 0, 0);
  private static final TimeOfFlight timeOfFlight = new TimeOfFlight(0);

  /** Constructs a new shooter object. */
  public Shooter() {
    shooterFollower.setInverted(true);
    shooterFollower.follow(shooterLeader);
    // TODO - tune PID
    shooterLeader.getPIDController().setP(0.04);
    shooterLeader.getPIDController().setI(0);
    shooterLeader.getPIDController().setD(0);
    pivot.getPIDController().setP(0.04);
    pivot.getPIDController().setI(0);
    pivot.getPIDController().setD(0);
    pivot.getPIDController().setFF(0.0002);
  }

  public static ShooterState getState() {
    return state;
  }

  public static void setState(ShooterState state) {
    Shooter.state = state;
  }

  @Override
  public void periodic() {
    switch (state) {
      default:
        break;
      case RECEIVE:
        // TODO - yet more values to tune
        indexer.set(0.1);
        if (timeOfFlight.getRange() < 5) setState(ShooterState.IDLE);
      // ... and more cases
    }
  }
}
