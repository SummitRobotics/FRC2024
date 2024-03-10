// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.SuperstructureDefault.StateChangeCommand;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/** Command factories for autos. */
public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /** One piece auto. */
  public static Command onePiece(Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> { superstructure.setState(SuperstructureState.IDLE); }),
        new StateChangeCommand(superstructure, intake, SuperstructureState.SPOOLING),
        new WaitCommand(1.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(1.5),
        new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
        // new InstantCommand(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 0)), drivetrain)
    );
  }

  /** Two piece auto. */
  public static Command twoPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.IDLE);
          intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("Two Piece"));
        }),
        new WaitUntilCommand(intake::atSetpoint),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.DOWN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
        new WaitCommand(1),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece"), true),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(2),
        new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        new WaitCommand(1.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }

  /** Traverses notes in an order that makes it extensible for later. */
  public static Command nPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.IDLE);
          intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("Two Piece"));
        }),
        new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < -29),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.DOWN);
        }),
        new WaitCommand(1),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("N Piece A"), true)
        ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.75),
        new ShooterAutomation(drivetrain, superstructure, intake),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("N Piece B"), false),
        new WaitCommand(0.75),
        new ShooterAutomation(drivetrain, superstructure, intake),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("N Piece C"), false),
        new WaitCommand(0.75),
        new ShooterAutomation(drivetrain, superstructure, intake)
    );
  }

  /** Two piece amp side. */
  public static Command twoPieceAmpSide(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.IDLE);
          intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("Two Piece"));
        }),
        new WaitUntilCommand(intake::atSetpoint),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.DOWN);
        }),
        new WaitCommand(1),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
          new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Amp Side A"), true)
        ),
        new WaitCommand(1.5),
        new ShooterAutomation(drivetrain, superstructure, intake)
    );
  }

  public static Command splineShoot(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return ShooterAutomation.splineWhileShooting(drivetrain, superstructure, intake, PathPlannerPath.fromPathFile("Shoot Test"));
  }

  /** Two piece side auto. */
  public static Command twoPieceOpenSide(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.IDLE);
          intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("Two Piece"));
        }),
        new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < -29),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.DOWN);
        }),
        new WaitCommand(1.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(1.5),
        new ParallelCommandGroup(
          new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new WaitCommand(1),
          new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece Open Side"), true)
        ),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(2),
        // new ParallelCommandGroup(
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("Two Piece Open Side Back"), false),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        // ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        new ShooterAutomation(drivetrain, superstructure, intake)
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
