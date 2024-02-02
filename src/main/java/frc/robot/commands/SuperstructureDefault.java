package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Default command for manual control of the superstructure. */
public class SuperstructureDefault extends Command {

  private Superstructure superstructure;
  private Intake intake;
  private BooleanSupplier receiveSupplier;
  private BooleanSupplier ampSupplier;
  private BooleanSupplier trapSupplier;
  // Shared between amp, trap, and speaker shoot; maybe shouldn't be
  private BooleanSupplier shootSupplier;
  private BooleanSupplier manualOverrideSupplier;
  private DoubleSupplier elevatorManualSupplier;
  private DoubleSupplier shooterManualSupplier;
  private DoubleSupplier indexerManualSupplier;
  private DoubleSupplier pivotManualSupplier;

  /** Creates a new SuperstructureDefault object. */
  public SuperstructureDefault(
      Superstructure superstructure,
      Intake intake,
      BooleanSupplier receiveSupplier,
      BooleanSupplier ampSupplier,
      BooleanSupplier trapSupplier,
      BooleanSupplier shootSupplier,
      BooleanSupplier manualOverrideSupplier,
      DoubleSupplier elevatorManualSupplier,
      DoubleSupplier shooterManualSupplier,
      DoubleSupplier indexerManualSupplier,
      DoubleSupplier pivotManualSupplier
  ) {
    addRequirements(superstructure);
    this.ampSupplier = ampSupplier;
    this.trapSupplier = trapSupplier;
    this.shootSupplier = shootSupplier;
    this.elevatorManualSupplier = elevatorManualSupplier;
    this.shooterManualSupplier = shooterManualSupplier;
    this.indexerManualSupplier = indexerManualSupplier;
    this.pivotManualSupplier = pivotManualSupplier;
  }

  @Override
  public void execute() {
    if (manualOverrideSupplier.getAsBoolean()) {
      superstructure.setState(SuperstructureState.MANUAL_OVERRIDE);
    } else if (receiveSupplier.getAsBoolean()) {
      superstructure.setState(SuperstructureState.RECEIVE);
    } else if (ampSupplier.getAsBoolean()) {
      superstructure.setState(SuperstructureState.AMP_READY);
    } else if (trapSupplier.getAsBoolean()) {
      superstructure.setState(SuperstructureState.TRAP_READY);
    } else if (shootSupplier.getAsBoolean()) {
      superstructure.setState(SuperstructureState.SPOOLING);
    }

    if (superstructure.getState() == SuperstructureState.MANUAL_OVERRIDE) {
      // Manual override could work inside or outside of the motion profiling.
      // Not sure which is better.
      // TODO - tune
      Superstructure.elevator.setGoal(new TrapezoidProfile.State(
          Superstructure.elevator.getGoal().position + 1 * elevatorManualSupplier.getAsDouble(), 0
      ));
      // In case this is unclear, setting the shooter's goal only affects pivot
      // because pivot is the only profiled degree of freedom
      Superstructure.shooter.setGoal(new TrapezoidProfile.State(
          Superstructure.shooter.getGoal().position + 1 * pivotManualSupplier.getAsDouble(), 0
      ));
    } else {
      // This does everything besides state transitions and stuff with shooter wheels.
      Superstructure.elevator.setGoal(superstructure.getState().elevatorEncoderVal);
      Superstructure.shooter.setGoal(superstructure.getState().pivotEncoderVal);
      Superstructure.shooter.setIndexer(superstructure.getState().indexerSpeed);
    }
    switch (superstructure.getState()) {
      case RECEIVE:
        if (Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.IDLE);
        }
        break;
      case AMP_READY:
        if (ampSupplier.getAsBoolean() && superstructure.atSetpoint()) {
          superstructure.setState(SuperstructureState.AMP_GO);
        }
        break;
      case AMP_GO:
        if (!Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.AMP_READY);
        }
      default:
        break;
    }
  }
}
