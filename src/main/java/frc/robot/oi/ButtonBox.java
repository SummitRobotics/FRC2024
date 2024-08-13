package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the Arduino Leonardo-driven button box. */
public class ButtonBox extends GenericHID {

  public enum Button {

    RECEIVE_PRESET(1, "RECEIVE"),
    AMP_PRESET(2, "AMP_PRESET"),
    SPEAKER_PRESET(3, "SPEAKER_PRESET"),
    TRAP_PRESET(4, "TRAP_PRESET"),
    SHOOT(6, "SHOOT"),
    PODIUM(5, "PODIUM_PRESET"),
    BLUE_SIGNAL(12, "BLUE_SIGNAL"),
    NO_SIGNAL(13, "NO_SIGNAL"),
    RED_SIGNAL(14, "RED_SIGNAL"),
    SPIT_FAR(7, "SPIT_FAR"),
    SPIT(8, "SPIT"),
    BABY_BIRD(9, "SOURCE_PICKUP");

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
    int ledIndex = button.index - 1;
    if (on) {
      this.ledState |= 1 << ledIndex;
    } else {
      this.ledState &= ~(1 << ledIndex);
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

  // TODO - is there a less verbose way to declare all these?
  public Trigger getReceivePreset() {
    return new Trigger(() -> getRawButton(Button.RECEIVE_PRESET.index));
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

  public Trigger getShoot() {
    return new Trigger(() -> getRawButton(Button.SHOOT.index));
  }

  public Trigger getPodiumPreset() {
    return new Trigger(() -> getRawButton(Button.PODIUM.index));
  }

  public Trigger getBlueSignal() {
    return new Trigger(() -> getRawButton(Button.BLUE_SIGNAL.index));
  }

  public Trigger getNoSignal() {
    return new Trigger(() -> getRawButton(Button.NO_SIGNAL.index));
  }

  public Trigger getRedSignal() {
    return new Trigger(() -> getRawButton(Button.RED_SIGNAL.index));
  }

  public Trigger getFarSpit() {
    return new Trigger(() -> getRawButton(Button.SPIT_FAR.index));
  }

  public Trigger getSpit() {
    return new Trigger(() -> getRawButton(Button.SPIT.index));
  }

  public Trigger getBabyBird() {
    return new Trigger(() -> getRawButton(Button.BABY_BIRD.index));
  }
}
