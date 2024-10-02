package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utilities.Functions;

/** Represents the intake subsystem. */
public class Intake extends SubsystemBase {

  // TODO - use SmartMotion profiling instead of WPILib

  /** Finite state machine options for the intake. */
  public enum IntakeState {
    IN(0.4, 0.4, "In"),
    IDLE(0, 0, "Idle"),
    OUT(-0.3, -0.3, "Out"),
    MANUAL_OVERRIDE(0, 0, "Manual Override");

    public double primaryRoller;
    public double secondaryRoller;
    public String name;
    private IntakeState(double primaryRoller, double secondaryRoller, String name) {
      this.name = name;
      this.primaryRoller = primaryRoller;
      this.secondaryRoller = secondaryRoller;
    }

    public String toString() {
      return this.name;
    }
  }

  private static IntakeState state;
  public static SuperstructureState pivotUpandDown;
  public static CANSparkMax primaryRoller;
  public static CANSparkMax secondaryRoller;

  /** Constructs a new Intake object. */
  public Intake() {
    // super(new TrapezoidProfile.Constraints(500, 200));
    primaryRoller = new CANSparkMax(7, MotorType.kBrushless);
    primaryRoller.restoreFactoryDefaults();
    secondaryRoller = new CANSparkMax(6, MotorType.kBrushless);
    Functions.setStatusFrames(primaryRoller);
    Functions.setStatusFrames(secondaryRoller);
    state = IntakeState.IDLE;
  }

  public IntakeState getState() {
    return state;
  }

  public void setState(IntakeState state) {
    Intake.state = state;
  }

  public void setPrimaryRoller(double val) {
    primaryRoller.set(val);
  }

  public void setSecondaryRoller(double val) {
    secondaryRoller.set(val);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("State", () -> state.name, null);
  }
}
