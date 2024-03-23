package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.RisingEdgeTrigger;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

/** Default command for the climb subsystem. */
public class ClimbDefault extends Command {

  Climb climb;
  Trigger autoClimb;
  Trigger leftUp;
  DoubleSupplier leftDown;
  Trigger rightUp;
  DoubleSupplier rightDown;
  Intake intake;
  RisingEdgeTrigger limit;
  private PIDController pid = new PIDController(0.05, 0, 0);
  private AHRS gyro;
  private boolean softLimitEnabled = false;

  /** Constructs a new ClimbDefault object. */
  public ClimbDefault(
      Climb climb,
      Intake intake,
      AHRS gyro,
      Trigger autoClimb,
      Trigger leftUp,
      Trigger rightUp,
      DoubleSupplier leftDown,
      DoubleSupplier rightDown,
      Trigger limit
  ) {
    this.climb = climb;
    // this.intake = intake;
    this.autoClimb = autoClimb;
    this.leftDown = leftDown;
    this.leftUp = leftUp;
    this.rightDown = rightDown;
    this.rightUp = rightUp;
    this.gyro = gyro;
    this.limit = new RisingEdgeTrigger(limit);
    addRequirements(climb);
  }

  @Override
  public void execute() {
    boolean limitBool = limit.get();
    if (limitBool) {
      if (!softLimitEnabled) {
        climb.armLeft.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climb.armRight.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climb.armLeft.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        climb.armRight.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
      } else {
        climb.armLeft.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        climb.armRight.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        climb.armLeft.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        climb.armRight.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
      }
      softLimitEnabled = !softLimitEnabled;
    }

    if (autoClimb.getAsBoolean() && climb.getState() != ClimbState.AUTO) {
      climb.setState(ClimbState.AUTO);
      CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
          new InstantCommand(() -> climb.setGoal(Climb.ABOVE_CHAIN)),
          new WaitCommand(2),
          new InstantCommand(() -> climb.setGoal(Climb.BELOW_CHAIN + pid.calculate(gyro.getRoll())))
            .until(pid::atSetpoint)
      ));
    } else if ((intake.getState() == IntakeState.MANUAL_OVERRIDE
        && climb.getState() != ClimbState.MANUAL_OVERRIDE)) {
      climb.setState(ClimbState.MANUAL_OVERRIDE);
      climb.armLeft.disable();
      climb.armRight.disable();
    } else if ((intake.getState() != IntakeState.MANUAL_OVERRIDE
        && climb.getState() == ClimbState.MANUAL_OVERRIDE)) {
      climb.setState(ClimbState.IDLE);
      // climb.armLeft.enable();
      climb.armLeft.disable();
      // climb.armRight.enable();
      climb.armRight.disable();
    }
    if (climb.getState() == ClimbState.MANUAL_OVERRIDE) {
      climb.setLeft((leftUp.getAsBoolean() ? 1 : 0) - (true ? leftDown.getAsDouble() : 0));
      climb.setRight((rightUp.getAsBoolean() ? 1 : 0) - (true ? rightDown.getAsDouble() : 0));
    }
  }
}
