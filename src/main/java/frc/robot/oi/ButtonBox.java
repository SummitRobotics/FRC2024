package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/** Represents the Arduino Leonardo-driven button box. */
public class ButtonBox extends GenericHID {

  public enum ButtonName {
    MANUAL_OVERRIDE(0),
    INTAKE_TOGGLE(1);

    public int index;

    ButtonName(int index) {
      this.index = index;
    }
  }

  private int ledState;

  public void LED(ButtonName button, boolean on) {
    if(on) {
      this.ledState |= 1 << button.index;
    }
    else {
      this.ledState &= ~(1 << button.index);
    }
  }

  public void sendMessage() {
    this.setOutputs(this.ledState & 0b111111111);
  }


  public ButtonBox(int port) {
    super(port);
  }

  public Trigger getManualOverride() {
    return new Trigger(() -> getRawButton(0));
  }
  
  public Trigger getAmpPreset() {
    return new Trigger(() -> getRawButton(3));
  }

  public Trigger getTrapPreset() {
    return new Trigger(() -> getRawButton(4));
  }

  public Trigger getspeakerPreset() {
    return new Trigger(() -> getRawButton(5));
  }

  public Trigger getArmAuto() {
    return new Trigger(() -> getRawButton(6));
  }

  public Trigger getArmAuto2() {
    return new Trigger(() -> getRawButton(7));
  }

  public Trigger getTrapAutoShoot() {
    return new Trigger(() -> getRawButton(8));
  }


}
