package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the Arduino Leonardo-driven button box. */
public class ButtonBox extends GenericHID {

  public enum Button {
    MANUAL_OVERRIDE(0, "MANUAL_OVERRIDE"),
    INTAKE_TOGGLE(1, "INTAKE_TOGGLE"),
    AMP_PRESET(2, "AMP_PRESET"),
    TRAP_PRESET(4, "TRAP_PRESET"),
    SPEAKER_PRESET(5, "SPEAKER_PRESET"),
    ARM_AUTO(6, "ARM_AUTO"),
    ARM_AUTO2(7, "ARM_AUTO2"),
    TRAP_AUTO_SHOOT(8, "TRAP_AUTO_SHOOT");

    public int index;
    public String name;

    Button(int index, String name) {
      this.index = index;
      this.name = name;
    }
  }

  private int ledState;
  private static int kLedMask = 0b111111111;

  public ButtonBox(int port) {
    super(port);
  }

  /**
   * Set the LED status for a given button. All Button LED's are updated in
   * SendMessage and persist until this method is updated.
   *
   * @param button The button to set LED status.
   * @param on     True to turn LED on, otherwise off.
   */
  public void LED(Button button, boolean on) {
    if (on) {
      this.ledState |= 1 << button.index;
    } else {
      this.ledState &= ~(1 << button.index);
    }
  }

  /**
   * Set all LEDs to either ON or OFF.
   *
   * @param on True to turn LEDs on, otherwise off.
   */
  public void AllLED(boolean on) {
    this.ledState = on ? kLedMask : 0;
  }

  /**
   * Sends the queued message to the ButtonBox arduino to decode.
   */
  public void sendMessage() {
    // TODO: Only updating LEDs. See Buttonbox/README.md for format of message
    this.setOutputs(this.ledState & kLedMask);
  }

  public Trigger getManualOverride() {
    return new Trigger(() -> getRawButton(Button.MANUAL_OVERRIDE.index));
  }

  public Trigger getIntakeToggle() {
    return new Trigger(() -> getRawButton(Button.INTAKE_TOGGLE.index));
  }

  public Trigger getAmpPreset() {
    return new Trigger(() -> getRawButton(Button.AMP_PRESET.index));
  }

  public Trigger getTrapPreset() {
    return new Trigger(() -> getRawButton(Button.TRAP_PRESET.index));
  }

  public Trigger getSpeakerPreset() {
    return new Trigger(() -> getRawButton(Button.SPEAKER_PRESET.index));
  }

  public Trigger getArmAuto() {
    return new Trigger(() -> getRawButton(Button.ARM_AUTO.index));
  }

  public Trigger getArmAuto2() {
    return new Trigger(() -> getRawButton(Button.ARM_AUTO2.index));
  }

  public Trigger getTrapAutoShoot() {
    return new Trigger(() -> getRawButton(Button.TRAP_AUTO_SHOOT.index));
  }
}
