package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;
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
  private Trigger shootConfirm;
  // TODO - tune
  private static final double TARGET_RPM = 10;

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
      DoubleSupplier pivotManualSupplier,
      Trigger shootConfirm
  ) {
    addRequirements(superstructure);
    this.intake = intake;
    this.superstructure = superstructure;
    this.receiveSupplier = new RisingEdgeTrigger(receiveSupplier);
    this.ampSupplier = new RisingEdgeTrigger(ampSupplier);
    this.trapSupplier = new RisingEdgeTrigger(trapSupplier);
    this.shootSupplier = new RisingEdgeTrigger(shootSupplier);
    this.manualOverrideSupplier = new RisingEdgeTrigger(manualOverrideSupplier);
    this.elevatorManualSupplier = elevatorManualSupplier;
    this.shooterManualSupplier = shooterManualSupplier;
    this.indexerManualSupplier = indexerManualSupplier;
    this.pivotManualSupplier = pivotManualSupplier;
    this.shootConfirm = shootConfirm;
    Superstructure.elevator.enable();
    // Superstructure.elevator.disable();
    Superstructure.shooter.enable();
  }

  @Override
  public void execute() {

    // Superstructure.shooter.recalibratePivot();

    // RisingEdgeTriggers have undesired behavior if polled twice per tick
    boolean amp = ampSupplier.get();
    boolean trap = trapSupplier.get();
    boolean shoot = shootSupplier.get();
    boolean receive = receiveSupplier.get();

    boolean mo = (intake.getState() == IntakeState.MANUAL_OVERRIDE && superstructure.getState() != SuperstructureState.MANUAL_OVERRIDE)
      || (intake.getState() != IntakeState.MANUAL_OVERRIDE && superstructure.getState() == SuperstructureState.MANUAL_OVERRIDE);

    if (mo) {
      if (superstructure.getState() != SuperstructureState.MANUAL_OVERRIDE) {
        Superstructure.elevator.disable();
        Superstructure.shooter.disable();
        superstructure.setState(SuperstructureState.MANUAL_OVERRIDE);
      } else {
        Superstructure.elevator.enable();
        // Superstructure.elevator.disable();
        Superstructure.shooter.enable();
        superstructure.setState(SuperstructureState.RECEIVE);
      }
    } else if (intake.getState() == IntakeState.DOWN) {
      if (receive) {
        superstructure.setState(SuperstructureState.RECEIVE);
      } else if (amp) {
        superstructure.setState(SuperstructureState.AMP_READY);
      } else if (trap) {
        superstructure.setState(SuperstructureState.TRAP_READY);
      } else if (shoot) {
        superstructure.setState(SuperstructureState.SPOOLING);
      }
    }

    if (superstructure.getState() != SuperstructureState.MANUAL_OVERRIDE) {
      // This does everything besides state transitions
      // Superstructure.elevator.setGoal(superstructure.getState().elevatorEncoderVal);
      Superstructure.Elevator.leader.getPIDController()
          .setReference(superstructure.getState().elevatorEncoderVal, ControlType.kPosition, 0, 1);
      Superstructure.shooter.setGoal(superstructure.getState().pivotEncoderVal);
      Superstructure.shooter.setIndexer(superstructure.getState().indexerSpeed);
      if (superstructure.getState() != SuperstructureState.SPOOLING
          && superstructure.getState() != SuperstructureState.SHOOTING) {
        // Superstructure.shooter.setShooterRpm(0);
        Superstructure.shooter.setShooter(0);
      } else {
        // Superstructure.shooter.setShooterRpm(TARGET_RPM);
        Superstructure.shooter.setShooter(1);
      }
    }
    switch (superstructure.getState()) {
      case RECEIVE:
        if (Superstructure.shooter.getToF()) {
          superstructure.setState(SuperstructureState.IDLE);
        }
        break;
      case AMP_READY:
        if (shootConfirm.getAsBoolean()) {
          superstructure.setState(SuperstructureState.AMP_GO);
        }
        break;
      case AMP_GO:
        // if (timer.get() > 3) {
          // superstructure.setState(SuperstructureState.AMP_READY);
        // }
        break;
      case TRAP_READY:
        if (shootConfirm.getAsBoolean()) {
          superstructure.setState(SuperstructureState.TRAP_GO);
        }
        break;
      case TRAP_GO:
        // if (timer.get() > 3) {
          // superstructure.setState(SuperstructureState.TRAP_READY);
        // }
        break;
      case SPOOLING:
        if (shootConfirm.getAsBoolean()) { //&& superstructure.atSetpoint()
            // && Superstructure.shooter.isSpooled()) {
          superstructure.setState(SuperstructureState.SHOOTING);
        }
        break;
      case SHOOTING:
        // if (timer.get() > 3) {
          // superstructure.setState(SuperstructureState.SPOOLING);
        // }
        break;
      case MANUAL_OVERRIDE:
        // Manual override could work inside or outside of the motion profiling.
        // Not sure which is better.
        // TODO - tune scaling factors for inputs
        // Superstructure.elevator.setGoal(3.5 * (elevatorManualSupplier.getAsDouble() + 1));
        Superstructure.Elevator.leader.set(elevatorManualSupplier.getAsDouble());
        // Superstructure.Elevator.follower.set(elevatorManualSupplier.getAsDouble());
        // System.out.println("Elevator leader set: " + Superstructure.Elevator.leader.get());
        // System.out.println("Elevator follower set" + Superstructure.Elevator.follower.get());
        // In case this is unclear, setting the shooter's goal only affects pivot
        // because pivot is the only profiled degree of freedom
        // Superstructure.shooter.setGoal(
            // Superstructure.shooter.getGoal().position + 40 * pivotManualSupplier.getAsDouble());
        Superstructure.Shooter.pivot.set(pivotManualSupplier.getAsDouble());
        Superstructure.shooter.setIndexer(indexerManualSupplier.getAsDouble());
        Superstructure.shooter.setShooter(shooterManualSupplier.getAsDouble());
        break;
      default:
        break;
    }
  }
}
