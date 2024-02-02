package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Drivetrain;

/** Follow a PathPlanner trajectory. */
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {
  // Assuming this is a method in your drive subsystem
  /** Constructor. */
  public FollowPathPlannerTrajectory(Drivetrain drivetrain, String pathname) {
    addCommands();
  }
}

