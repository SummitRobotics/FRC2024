package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import java.util.function.DoubleSupplier;

/** Default command for manual control of the superstructure.
 * We might need to separate manual overrides into a different file so we can use PrioritizedAxes
 * like in previous years.
 * The alternative is changing ArcadeDrive based on elevator / shooter state.
*/
public class SuperstructureDefault extends Command {

  private Superstructure superstructure;
  private Intake intake;
  private RisingEdgeTrigger receiveSupplier;
  private RisingEdgeTrigger ampSupplier;
  private RisingEdgeTrigger trapSupplier;
  // Shared between amp, trap, and speaker shoot; maybe shouldn't be
  private RisingEdgeTrigger shootSupplier;
  private RisingEdgeTrigger manualOverrideSupplier;
  private DoubleSupplier elevatorManualSupplier;
  private DoubleSupplier shooterManualSupplier;
  private DoubleSupplier indexerManualSupplier;
  private DoubleSupplier pivotManualSupplier;
  // TODO - tune
  private static final double TARGET_RPM = 0;

  /** Creates a new SuperstructureDefault object. */
  public SuperstructureDefault(
      Superstructure superstructure,
      Intake intake,
      Trigger receiveSupplier,
      Trigger ampSupplier,
      Trigger trapSupplier,
      Trigger shootSupplier,
      Trigger manualOverrideSupplier,
      DoubleSupplier elevatorManualSupplier,
      DoubleSupplier shooterManualSupplier,
      DoubleSupplier indexerManualSupplier,
      DoubleSupplier pivotManualSupplier
  ) {
    addRequirements(superstructure);
    this.intake = intake;
    this.receiveSupplier = new RisingEdgeTrigger(receiveSupplier);
    this.ampSupplier = new RisingEdgeTrigger(ampSupplier);
    this.trapSupplier = new RisingEdgeTrigger(trapSupplier);
    this.shootSupplier = new RisingEdgeTrigger(shootSupplier);
    this.manualOverrideSupplier = new RisingEdgeTrigger(manualOverrideSupplier);
    this.elevatorManualSupplier = elevatorManualSupplier;
    this.shooterManualSupplier = shooterManualSupplier;
    this.indexerManualSupplier = indexerManualSupplier;
    this.pivotManualSupplier = pivotManualSupplier;
  }

  @Override
  public void execute() {
    if (manualOverrideSupplier.get()) {
      superstructure.setState(SuperstructureState.MANUAL_OVERRIDE);
    } else if (intake.getState() == IntakeState.DOWN) {
      if (receiveSupplier.get()) {
        superstructure.setState(SuperstructureState.RECEIVE);
      } else if (ampSupplier.get()) {
        superstructure.setState(SuperstructureState.AMP_READY);
      } else if (trapSupplier.get()) {
        superstructure.setState(SuperstructureState.TRAP_READY);
      } else if (shootSupplier.get()) {
        superstructure.setState(SuperstructureState.SPOOLING);
      }
    }

    if (superstructure.getState() == SuperstructureState.MANUAL_OVERRIDE) {
      // Manual override could work inside or outside of the motion profiling.
      // Not sure which is better.
      // TODO - tune scaling factors for inputs
      Superstructure.elevator.setGoal(new TrapezoidProfile.State(
          Superstructure.elevator.getGoal().position + 1 * elevatorManualSupplier.getAsDouble(), 0
      ));
      // In case this is unclear, setting the shooter's goal only affects pivot
      // because pivot is the only profiled degree of freedom
      Superstructure.shooter.setGoal(new TrapezoidProfile.State(
          Superstructure.shooter.getGoal().position + 1 * pivotManualSupplier.getAsDouble(), 0
      ));
      Superstructure.shooter.setIndexer(indexerManualSupplier.getAsDouble());
      Superstructure.shooter.setShooter(shooterManualSupplier.getAsDouble());
    } else {
      // This does everything besides state transitions
      Superstructure.elevator.setGoal(superstructure.getState().elevatorEncoderVal);
      Superstructure.shooter.setGoal(superstructure.getState().pivotEncoderVal);
      Superstructure.shooter.setIndexer(superstructure.getState().indexerSpeed);
      if (superstructure.getState() != SuperstructureState.SPOOLING
          && superstructure.getState() != SuperstructureState.SHOOTING) {
        Superstructure.shooter.setShooterRpm(0);
      } else {
        Superstructure.shooter.setShooterRpm(TARGET_RPM);
      }
    }
    switch (superstructure.getState()) {
      case RECEIVE:
        if (Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.IDLE);
        }
        break;
      case AMP_READY:
        if (ampSupplier.get() && superstructure.atSetpoint()) {
          superstructure.setState(SuperstructureState.AMP_GO);
        }
        break;
      case AMP_GO:
        if (!Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.AMP_READY);
        }
        break;
      case TRAP_READY:
        if (trapSupplier.get() && superstructure.atSetpoint()) {
          superstructure.setState(SuperstructureState.TRAP_GO);
        }
        break;
      case TRAP_GO:
        if (!Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.TRAP_READY);
        }
        break;
      case SPOOLING:
        if (shootSupplier.get() && superstructure.atSetpoint()
            && Superstructure.shooter.isSpooled()) {
          superstructure.setState(SuperstructureState.SHOOTING);
        }
        break;
      case SHOOTING:
        if (!Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.SPOOLING);
        }
        break;
      default:
        break;
    }
  }
}
