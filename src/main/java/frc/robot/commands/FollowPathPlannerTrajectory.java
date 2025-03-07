package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;

/** Follow a PathPlanner trajectory.
 * Based on docs at https://pathplanner.dev/pplib-follow-a-single-path.html#manually-create-path-following-commands.
*/
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {

  /** Constructor. */
  public FollowPathPlannerTrajectory(Swerve drivetrain, PathPlannerPath path) {
    addCommands(
      new InstantCommand(() -> {
        // if (resetPose) drivetrain.setPose(path.getPreviewStartingHolonomicPose());
        PPLibTelemetry.setCurrentPath(path);
      }),
      new FollowPathHolonomic(
        path,
        drivetrain::getPose,
        drivetrain.getConstellation()::chassisSpeeds,
        drivetrain::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(2, 0, 0.05),
            new PIDConstants(2, 0, 0.05),
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
  }

  public FollowPathPlannerTrajectory(Swerve drivetrain, PathPlannerPath path, boolean doRotate) {
    addCommands(
      new InstantCommand(() -> {
        // if (resetPose) drivetrain.setPose(path.getPreviewStartingHolonomicPose());
        PPLibTelemetry.setCurrentPath(path);
      }),
      new FollowPathHolonomic(
        path,
        drivetrain::getPose,
        drivetrain.getConstellation()::chassisSpeeds,
          (ChassisSpeeds speeds) -> {
            if (!doRotate) {
              drivetrain.drive(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, drivetrain.getCurrentVelocity().omegaRadiansPerSecond));
            } else {
              drivetrain.drive(speeds);
            }
          },
          new HolonomicPathFollowerConfig(
              new PIDConstants(2, 0, 0.05),
              new PIDConstants(2, 0, 0.05),
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
  }
}
