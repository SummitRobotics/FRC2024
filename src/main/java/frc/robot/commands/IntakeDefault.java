package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
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
  private Superstructure superstructure;
  private RisingEdgeTrigger manualOverrideSupplier;
  private RisingEdgeTrigger pivotUpandDown;
  DoubleSupplier manualPivot;
  DoubleSupplier manualRoller;

  /** Constructor for IntakeDefault. */
  public IntakeDefault(
      Intake intake,
      Superstructure superstructure,
      Trigger manualOverrideSupplier,
      Trigger pivotUpandDown,
      DoubleSupplier manualPivot,
      DoubleSupplier manualRoller
  ) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.manualPivot = manualPivot;
    this.manualRoller = manualRoller;
    this.manualOverrideSupplier = new RisingEdgeTrigger(manualOverrideSupplier);
    this.pivotUpandDown = new RisingEdgeTrigger(pivotUpandDown);
    intake.enable();
    addRequirements(intake);
  }

  @Override
  public void execute() {

    // We shouldn't need to do this, but intake is moving of its own accord in auto.
    if (DriverStation.isAutonomous()) {
      intake.setState(IntakeState.DOWN);
    }

    if (manualOverrideSupplier.get()) {
      if (intake.getState() != IntakeState.MANUAL_OVERRIDE) {
        intake.disable();
        intake.setState(IntakeState.MANUAL_OVERRIDE);
      } else {
        intake.enable();
        intake.setState(IntakeState.DOWN);
      }
    }
    
    if (intake.getState() != IntakeState.MANUAL_OVERRIDE
        && pivotUpandDown.get()/* && superstructure.atSetpoint()*/) {
      intake.setState(intake.getState() == IntakeState.DOWN ? IntakeState.UP : IntakeState.DOWN);
    }

    intake.setGoal(intake.getState().pivot);
    intake.setRoller(intake.getState().roller);
    if (intake.getState() == IntakeState.MANUAL_OVERRIDE) {
      intake.setPivot(manualPivot.getAsDouble());
      intake.setRoller(manualRoller.getAsDouble());
    }
  }
}
