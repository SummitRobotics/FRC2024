package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Climb;

/** Default command for the climb subsystem. */
public class ClimbDefault extends Command {

  Climb climb;
  RisingEdgeTrigger autoClimb;
  Trigger leftUp;
  Trigger leftDown;
  Trigger rightUp;
  Trigger rightDown;

  /** Constructs a new ClimbDefault object. */
  public ClimbDefault(
      Climb climb,
      Trigger autoClimb,
      Trigger leftUp,
      Trigger rightUp,
      Trigger leftDown,
      Trigger rightDown
  ) {
    this.climb = climb;
    this.autoClimb = new RisingEdgeTrigger(autoClimb);
    this.leftDown = leftDown;
    this.leftUp = leftUp;
    this.rightDown = rightDown;
    this.rightUp = rightUp;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    // TODO - tune manual override speeds
    climb.armLeft.setGoal(climb.armLeft.getGoal().position
        + (leftUp.getAsBoolean() ? 1 : 0) - (leftDown.getAsBoolean() ? 1 : 0));
    climb.armRight.setGoal(climb.armRight.getGoal().position
        + (rightUp.getAsBoolean() ? 1 : 0) - (rightDown.getAsBoolean() ? 1 : 0));
  }

  @Override
  public boolean isFinished() {
    return autoClimb.get();
  }

  @Override
  public void end(final boolean interrupted) {
    climb.armLeft.setGoal(climb.armLeft.getGoal().position);
    climb.armRight.setGoal(climb.armRight.getGoal().position);
  }
}
