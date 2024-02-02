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

  /** Reads from the RisingEdgeTrigger. */
  public boolean get() {
    lastState = currentState;
    currentState = trigger.getAsBoolean();
    return currentState && !lastState;
  }

  public Trigger getTrigger() {
    return trigger;
  }
}
