package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.Functions;

/** Adds deadzones to the stock WPILib controller class. */
public class Controller extends XboxController {

  private final double DEADZONE = 0.05;

  public Controller(int id) {
    super(id);
  }

  @Override
  public double getLeftX() {
    return deadzone(super.getLeftX());
  }

  public double getLeftY() {
    return deadzone(super.getLeftY());
  }

  public double getRightX() {
    return deadzone(super.getRightX());
  }

  public double getRightY() {
    return deadzone(super.getRightY());
  }

  public double getRightTrigger() {
    return deadzone(super.getRightTriggerAxis());
  }

  public double getLeftTrigger() {
    return deadzone(super.getLeftTriggerAxis());
  }

  // TODO - less verbose way to get triggers
  public Trigger xButton() {
    return new Trigger(() -> getXButton());
  }

  public Trigger yButton() {
    return new Trigger(() -> getYButton());
  }

  public Trigger aButton() {
    return new Trigger(() -> getAButton());
  }

  public Trigger bButton() {
    return new Trigger(() -> getBButton());
  }

  public Trigger leftBumpTrigger() {
    return new Trigger(() -> getLeftBumper());
  }

  public Trigger rightBumpTrigger() {
    return new Trigger(() -> getRightBumper());
  }

  private double deadzone(double in) {
    return Functions.withinTolerance(in, 0, DEADZONE) ? 0
      // Rescale so values directly after deadzone start at 0
      : (1 + DEADZONE) * in - Math.copySign(DEADZONE, in);
  }
}
