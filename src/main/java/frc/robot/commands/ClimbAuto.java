package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utilities.Functions;

/** Automated climb sequence. */
public class ClimbAuto extends SequentialCommandGroup {

  // TODO - finish with interruptability / vision alignment and tune

  /** Constructor. */
  public ClimbAuto(Climb climb, Superstructure superstructure, AHRS gyro) {
    addCommands(
      new SequentialCommandGroup(
          new InstantCommand(() -> climb.setGoal(Climb.HEIGHT)),
          new WaitUntilCommand(() -> climb.atSetpoint()),
          new InstantCommand(() -> climb.setGoal(Climb.OFF_GROUND)),
          new WaitUntilCommand(() -> climb.atSetpoint())
      ).until(() -> (climb.armLeft.getCurrent() && climb.armRight.getCurrent()
          && Functions.withinTolerance(gyro.getRoll(), 0, 5))),
        new InstantCommand(() -> {
          climb.setGoal(0);
          superstructure.setState(SuperstructureState.TRAP_READY);
        }),
        new WaitUntilCommand(() -> climb.atSetpoint() && superstructure.atSetpoint()
            && Functions.withinTolerance(gyro.getRoll(), 0, 5)),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.TRAP_GO))
    );
    addRequirements(climb);
  }
}
