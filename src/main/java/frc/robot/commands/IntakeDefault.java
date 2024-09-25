package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import java.util.function.DoubleSupplier;

/** Default command for the intake. */
public class IntakeDefault extends Command {

  private Intake intake;
  private RisingEdgeTrigger manualOverrideSupplier;
  DoubleSupplier manualPrimary;
  DoubleSupplier manualSecondary;

  /** Constructor for IntakeDefault. */
  public IntakeDefault(
      Intake intake,
      Superstructure superstructure,
      Trigger manualOverrideSupplier,
      DoubleSupplier manualPrimary,
      DoubleSupplier manualSecondary
  ) {
    this.intake = intake;
    this.manualPrimary = manualPrimary;
    this.manualSecondary = manualSecondary;
    this.manualOverrideSupplier = new RisingEdgeTrigger(manualOverrideSupplier);
    addRequirements(intake);
  }

  @Override
  public void execute() {

    if (manualOverrideSupplier.get()) {
      if (intake.getState() != IntakeState.MANUAL_OVERRIDE) {
        intake.setState(IntakeState.MANUAL_OVERRIDE);
      } else {
        intake.setState(IntakeState.IDLE);
      }
    }

    if (intake.getState() == IntakeState.MANUAL_OVERRIDE) {
      intake.setPrimaryRoller(manualPrimary.getAsDouble());
      intake.setSecondaryRoller(manualSecondary.getAsDouble());
    } else {
      intake.setPrimaryRoller(intake.getState().primaryRoller);
      intake.setSecondaryRoller(intake.getState().secondaryRoller);
    }
  }
}
