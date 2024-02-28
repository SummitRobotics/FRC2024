package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

/** Follow a PathPlanner trajectory.
 * Based on docs at https://pathplanner.dev/pplib-follow-a-single-path.html#manually-create-path-following-commands.
*/
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {
  // Assuming this is a method in your drive subsystem
  /** Constructor. */
  public FollowPathPlannerTrajectory(Swerve drivetrain, PathPlannerPath path) {
    drivetrain.setPose(path.getPreviewStartingHolonomicPose());
    addCommands(
      new FollowPathHolonomic(
        path,
        drivetrain::getPose,
        drivetrain.getConstellation()::chassisSpeeds,
          (ChassisSpeeds speeds) -> {
            drivetrain.drive(
              new ChassisSpeeds(-speeds.vxMetersPerSecond,
                -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond));
          },
          new HolonomicPathFollowerConfig(
              new PIDConstants(0.05, 0, 0),
              new PIDConstants(0.025, 0, 0),
          4, // meters per second
          0.45,
              new ReplanningConfig()
        ),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
        drivetrain
    ));
    // addRequirements(drivetrain);
  }
}
