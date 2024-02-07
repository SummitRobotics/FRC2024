package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Drivetrain;

/** Follow a PathPlanner trajectory.
 * Based on docs at https://pathplanner.dev/pplib-follow-a-single-path.html#manually-create-path-following-commands.
*/
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {
  // Assuming this is a method in your drive subsystem
  /** Constructor. */
  public FollowPathPlannerTrajectory(Drivetrain drivetrain, String pathname) {
    addCommands(
      new FollowPathHolonomic(
        PathPlannerPath.fromPathFile(pathname),
        drivetrain::getPose,
        drivetrain.getConstellation()::chassisSpeeds,
        drivetrain::drive,
          new HolonomicPathFollowerConfig(
              new PIDConstants(5, 0, 0),
              new PIDConstants(5, 0, 0),
          4.5,
          0.4,
              new ReplanningConfig()
        ),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }
    ));
    addRequirements(drivetrain);
  }
}
