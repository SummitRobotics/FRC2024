package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

/** Default command for the climb subsystem. */
public class ClimbDefault extends Command {

  Climb climb;
  RisingEdgeTrigger manualOverride;
  Trigger autoClimb;
  Trigger leftUp;
  Trigger leftDown;
  Trigger rightUp;
  Trigger rightDown;
  RisingEdgeTrigger down;
  RisingEdgeTrigger mid;
  RisingEdgeTrigger up;
  boolean overrideActive = false;
  Intake intake;

  /** Constructs a new ClimbDefault object. */
  public ClimbDefault(
      Climb climb,
      Intake intake,
      Trigger manualOverride,
      Trigger autoClimb,
      Trigger leftUp,
      Trigger rightUp,
      Trigger leftDown,
      Trigger rightDown,
      Trigger down,
      Trigger mid,
      Trigger up
  ) {
    this.climb = climb;
    this.intake = intake;
    this.manualOverride = new RisingEdgeTrigger(manualOverride);
    this.autoClimb = autoClimb;
    this.leftDown = leftDown;
    this.leftUp = leftUp;
    this.rightDown = rightDown;
    this.rightUp = rightUp;
    this.down = new RisingEdgeTrigger(down);
    this.mid = new RisingEdgeTrigger(mid);
    this.up = new RisingEdgeTrigger(up);
    addRequirements(climb);
  }

  @Override
  public void execute() {

    // Rising edge trigger bug workaround
    boolean downBool = down.get();
    boolean midBool = mid.get();
    boolean upBool = up.get();
    if ((intake.getState() == IntakeState.MANUAL_OVERRIDE && !overrideActive)) {
      overrideActive = !overrideActive;
      climb.armLeft.disable();
      climb.armRight.disable();
    } else if ((intake.getState() != IntakeState.MANUAL_OVERRIDE && overrideActive)) {
      overrideActive = !overrideActive;
      climb.armLeft.enable();
      climb.armRight.enable();
    }
    if (overrideActive) {
      // TODO - tune manual override speeds
      // climb.armLeft.setGoal(climb.armLeft.getGoal().position
          // + (leftUp.getAsBoolean() ? 1 : 0) - (leftDown.getAsBoolean() ? 1 : 0));
      // climb.armRight.setGoal(climb.armRight.getGoal().position
          // + (rightUp.getAsBoolean() ? 1 : 0) - (rightDown.getAsBoolean() ? 1 : 0));
      climb.setLeft((leftUp.getAsBoolean() ? 1 : 0) - (leftDown.getAsBoolean() ? 1 : 0));
      climb.setRight((rightUp.getAsBoolean() ? 1 : 0) - (rightDown.getAsBoolean() ? 1 : 0));
    } else {
      if (downBool) {
        climb.setGoal(0);
      } else if (midBool) {
        climb.setGoal(Climb.OFF_GROUND);
      } else if (upBool) {
        climb.setGoal(Climb.HEIGHT);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(final boolean interrupted) {
    climb.armLeft.setGoal(climb.armLeft.getGoal().position);
    climb.armRight.setGoal(climb.armRight.getGoal().position);
  }

}
