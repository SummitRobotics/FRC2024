package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** Rising edge trigger - true only once even if the button is held down.
 * Good for toggles.
 */
public class RisingEdgeTrigger {
  private boolean lastState = false;
  private boolean currentState = false;
  private final Trigger trigger;

  public RisingEdgeTrigger(Trigger trigger) {
    this.trigger = trigger;
  }

  public RisingEdgeTrigger(BooleanSupplier trigger) {
    this(new Trigger(trigger));
  }

  /** Reads from the RisingEdgeTrigger.
   * The intent was to return true for a single tick before returning to false again.
   * This isn't the case if polled multiple times per tick.
   * TODO - fix this
  */
  public boolean get() {
    lastState = currentState;
    currentState = trigger.getAsBoolean();
    return currentState && !lastState;
  }

  public Trigger getTrigger() {
    return trigger;
  }
}
