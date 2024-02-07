package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utilities.Functions;


public class Controller extends XboxController {


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
    return deadzone(super.getLeftX());
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

  private double deadzone(double in) {
    return !Functions.withinTolerance(in, 0, 0.05) ? in : 0;
  }

}
