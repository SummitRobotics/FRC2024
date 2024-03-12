package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.ButtonBox;
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
  private ButtonBox buttonBox;
  private RisingEdgeTrigger receiveSupplier;
  private RisingEdgeTrigger ampSupplier;
  private RisingEdgeTrigger trapSupplier;
  private RisingEdgeTrigger podiumSupplier;
  // Shared between amp, trap, and speaker shoot; maybe shouldn't be
  private RisingEdgeTrigger shootSupplier;
  private DoubleSupplier elevatorManualSupplier;
  private DoubleSupplier shooterManualSupplier;
  private DoubleSupplier indexerManualSupplier;
  private DoubleSupplier pivotManualSupplier;
  private Trigger shootConfirm;
  private RisingEdgeTrigger spitSupplier;
  // TODO - tune
  // private static final double TARGET_RPM = 10;
  private Timer timer = new Timer();

  public static class StateChangeCommand extends SequentialCommandGroup {
    public StateChangeCommand(
        Superstructure superstructure,
        Intake intake,
        SuperstructureState state
    ) {
      addCommands(
        new InstantCommand(() -> {
          intake.setState(IntakeState.MID);
        }),
        new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < -29),
        new InstantCommand(() -> {
          superstructure.setState(state);
        }),
        new WaitCommand(0.6),
        new InstantCommand(() -> {
          intake.setState(state == SuperstructureState.SPOOLING ? IntakeState.MID : IntakeState.UP);
        })
      );
    }
  }

  /** Creates a new SuperstructureDefault object. */
  public SuperstructureDefault(
      Superstructure superstructure,
      Intake intake,
      ButtonBox buttonBox,
      Trigger receiveSupplier,
      Trigger ampSupplier,
      Trigger trapSupplier,
      Trigger shootSupplier,
      Trigger podiumSupplier,
      DoubleSupplier elevatorManualSupplier,
      DoubleSupplier shooterManualSupplier,
      DoubleSupplier indexerManualSupplier,
      DoubleSupplier pivotManualSupplier,
      Trigger shootConfirm,
      Trigger spitSupplier
  ) {
    addRequirements(superstructure);
    this.intake = intake;
    this.buttonBox = buttonBox;
    this.superstructure = superstructure;
    this.receiveSupplier = new RisingEdgeTrigger(receiveSupplier);
    this.ampSupplier = new RisingEdgeTrigger(ampSupplier);
    this.trapSupplier = new RisingEdgeTrigger(trapSupplier);
    this.shootSupplier = new RisingEdgeTrigger(shootSupplier);
    this.podiumSupplier = new RisingEdgeTrigger(podiumSupplier);
    this.spitSupplier = new RisingEdgeTrigger(spitSupplier);
    this.elevatorManualSupplier = elevatorManualSupplier;
    this.shooterManualSupplier = shooterManualSupplier;
    this.indexerManualSupplier = indexerManualSupplier;
    this.pivotManualSupplier = pivotManualSupplier;
    this.shootConfirm = shootConfirm;
    // Superstructure.elevator.enable();
    // Superstructure.elevator.disable();
    // Superstructure.shooter.enable();
    timer.stop();
  }

  @Override
  public void initialize() {
    // Superstructure.shooter.recalibratePivot();
  }

  @Override
  public void execute() {
    // Superstructure.shooter.recalibratePivot();
    // RisingEdgeTriggers have undesired behavior if polled twice per tick
    boolean amp = ampSupplier.get();
    boolean trap = trapSupplier.get();
    boolean shoot = shootSupplier.get();
    boolean receive = receiveSupplier.get();
    boolean podium = podiumSupplier.get();
    boolean spit = spitSupplier.get();

    SuperstructureState superState = superstructure.getState();

    boolean mo = (intake.getState() == IntakeState.MANUAL_OVERRIDE && superState != SuperstructureState.MANUAL_OVERRIDE)
      || (intake.getState() != IntakeState.MANUAL_OVERRIDE && superState == SuperstructureState.MANUAL_OVERRIDE);

    if (mo) {
      if (superState != SuperstructureState.MANUAL_OVERRIDE) {
        // Superstructure.elevator.disable();
        // Superstructure.shooter.disable();
        superstructure.setState(SuperstructureState.MANUAL_OVERRIDE);
      } else {
        // Superstructure.elevator.enable();
        // Superstructure.shooter.enable();
        CommandScheduler.getInstance()
            .schedule(new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE));
      }
    } else if (superstructure.atSetpoint()) {
      if (receive) {
        CommandScheduler.getInstance().schedule(
            new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE));
      } else if (amp) {
        CommandScheduler.getInstance().schedule(
            new StateChangeCommand(superstructure, intake, SuperstructureState.AMP_READY));
      } else if (trap) {
        CommandScheduler.getInstance().schedule(
            new StateChangeCommand(superstructure, intake, SuperstructureState.TRAP_READY));
      } else if (shoot) {
        CommandScheduler.getInstance().schedule(
            new StateChangeCommand(superstructure, intake, SuperstructureState.SPOOLING));
      } else if (podium) {
        CommandScheduler.getInstance().schedule(
          new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY));
      } else if (spit) {
        CommandScheduler.getInstance().schedule(
          new StateChangeCommand(superstructure, intake, SuperstructureState.EJECT_READY));
      }
    }

    if (superState != SuperstructureState.MANUAL_OVERRIDE) {
      // This does everything besides state transitions
      Superstructure.shooter.setShooter(superState.shooterSpeed);
      if (superState != SuperstructureState.VARIABLE_READY && superState != SuperstructureState.VARIABLE_GO) {
        Superstructure.Elevator.leader.getPIDController()
            .setReference(superState.elevatorEncoderVal, ControlType.kPosition, 0, 2.0);
        // Superstructure.shooter.setGoal(superState.pivotEncoderVal);
        Superstructure.Shooter.pivot.getPIDController().setReference(superState.pivotEncoderVal, ControlType.kPosition);
        Superstructure.shooter.setIndexer(superState.indexerSpeed);
      } else {
        Superstructure.Elevator.leader.getPIDController()
          .setReference(Superstructure.variableElevator, ControlType.kPosition, 0, 2.0);
        // Superstructure.shooter.setGoal(Superstructure.variablePivot);
        Superstructure.Shooter.pivot.getPIDController().setReference(Superstructure.variablePivot, ControlType.kPosition);
        Superstructure.shooter.setIndexer(Superstructure.variableIndexer);
      }
    }

    buttonBox.LED(ButtonBox.Button.RECEIVE_PRESET, false);
    buttonBox.LED(ButtonBox.Button.AMP_PRESET, false);
    buttonBox.LED(ButtonBox.Button.SPEAKER_PRESET, false);
    buttonBox.LED(ButtonBox.Button.SHOOT, false);

    if (superState != SuperstructureState.RECEIVE) {
      timer.stop();
      timer.reset();
    }

    switch (superState) {
      case RECEIVE:
        if (Superstructure.shooter.getToF()) {
          timer.restart();
        }
        if (timer.get() > 0.15) {
          superstructure.setState(SuperstructureState.IDLE);
          intake.setState(IntakeState.UP);
        }
        buttonBox.LED(ButtonBox.Button.RECEIVE_PRESET, true);
        break;
      case AMP_READY:
        if (shootConfirm.getAsBoolean()) {
          superstructure.setState(SuperstructureState.AMP_GO);
        }
        buttonBox.LED(ButtonBox.Button.AMP_PRESET, true);
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
        buttonBox.LED(ButtonBox.Button.SHOOT, true);
        break;
      case SHOOTING:
        // if (timer.get() > 3) {
          // superstructure.setState(SuperstructureState.SPOOLING);
        // }
        buttonBox.LED(ButtonBox.Button.SPEAKER_PRESET, true);
        break;
      case PODIUM_READY:
        if (shootConfirm.getAsBoolean()) {
          superstructure.setState(SuperstructureState.PODIUM_GO);
        }
        // buttonBox.LED(ButtonBox.Button.SHOOT, true);
        break;
      case EJECT_READY:
        if (shootConfirm.getAsBoolean()) {
          superstructure.setState(SuperstructureState.EJECT_GO);
        }
      case PODIUM_GO:
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
        Superstructure.Shooter.pivot.set(-pivotManualSupplier.getAsDouble());
        Superstructure.shooter.setIndexer(indexerManualSupplier.getAsDouble());
        Superstructure.shooter.setShooter(shooterManualSupplier.getAsDouble());
        break;
      default:
        break;
    }
  }
}
