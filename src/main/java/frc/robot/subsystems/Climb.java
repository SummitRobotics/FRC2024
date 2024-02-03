package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.GoodTrapezoidProfileSubsystem;

/** Represents the climb subsystem. */
public class Climb extends SubsystemBase {

  private Arm armLeft = new Arm();
  private ElevatorFeedforward leftFeedforward = new ElevatorFeedforward(0, 0, 0);
  private Arm armRight = new Arm();
  private ElevatorFeedforward rightFeedforward = new ElevatorFeedforward(0, 0, 0);

  public Climb() {

  }

  private class Arm extends GoodTrapezoidProfileSubsystem {
    public Arm() {
      super(new TrapezoidProfile.Constraints(0, 0));
    }

    @Override
    protected void useState(State state) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'useState'");
    }
  }
}
