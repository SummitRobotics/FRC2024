// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuperstructureDefault.StateChangeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;

// TODO - correct redundancy; most of these autos only differ by PathPlanner path names.
// Test and convert things over to the AutoFactory class

/** Command factories for autos. */
public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /** One piece auto. */
  public static Command onePiece(Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.IN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE))
    );
  }

  /** Two piece auto. */
  // TODO - retool after intake change
  public static Command twoPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.IN);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(intake::atSetpoint),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.IN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
        new WaitCommand(1),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("TwoPiece")),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.5),
        new ShooterAutomation(drivetrain, superstructure, intake)
        // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }

  /*public static Command threePiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.IN);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(intake::atSetpoint),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.IN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
        new WaitCommand(1),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("TwoPiece")),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.5),
        new ShooterAutomation(drivetrain, superstructure, intake),
        new WaitCommand(0.5),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceC")),
        new WaitCommand(0.5),
        new ShooterAutomation(drivetrain, superstructure, intake)
        // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }*/

  public static Command ampSideThree(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.IN);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(intake::atSetpoint),
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.IN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
        new WaitCommand(1),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideA")),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.5),
        new ShooterAutomation(drivetrain, superstructure, intake),
        new WaitCommand(0.5),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideB")),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.5),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideC")),
        new InstantCommand(drivetrain::stop, drivetrain),
        new WaitCommand(0.5),
        new ShooterAutomation(drivetrain, superstructure, intake)

        // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }

  private static class AutoFactory extends SequentialCommandGroup {
    public AutoFactory(Swerve drivetrain, Superstructure superstructure, Intake intake, String... subsequentPaths) {
      addCommands(
        new InstantCommand(() -> {
          superstructure.setState(SuperstructureState.SPOOLING);
          intake.setState(IntakeState.IN);
        }),
        new WaitCommand(0.4),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(0.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE))
      );

      for (String pathName : subsequentPaths) {
        addCommands(
          new WaitCommand(0.1),
          new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile(pathName)),
          new InstantCommand(drivetrain::stop, drivetrain),
          new WaitCommand(0.75),
          new ShooterAutomation(drivetrain, superstructure, intake)
        );
      }
    }
  }

  public static Command fourPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new AutoFactory(drivetrain, superstructure, intake, "NPieceA", "NPieceB", "OtherNPieceC");
  }

  public static Command threePiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new AutoFactory(drivetrain, superstructure, intake, "TwoPiece", "NPieceC");
  }

  public static Command twoPieceAmp(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new AutoFactory(drivetrain, superstructure, intake, "TwoPiece", "NPieceC");
  }

  public static Command twoPieceFar(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new AutoFactory(drivetrain, superstructure, intake, "TwoPiece", "NPieceC");
  }

  /** Traverses notes in an order that makes it extensible for later. */
  // TODO - retool after intake change
  // public static Command nPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceA"))
        // ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceB")),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceC")),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake)
    // );
  // }

  // TODO - retool after intake change
  // public static Command far(Swerve drivetrain, Superstructure superstructure, Intake intake, boolean outer) {
    // return new SequentialCommandGroup(
      // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("FarA"))
        // ),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, outer ? PathPlannerPath.fromPathFile("FarB") : PathPlannerPath.fromPathFile("FarB2")),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SPOOLING)),
        // new FollowPathPlannerTrajectory(drivetrain, outer ? PathPlannerPath.fromPathFile("FarC") : PathPlannerPath.fromPathFile("FarC2")),
        // new ShooterAutomation(drivetrain, superstructure, intake)
    // );
  // }

  /** Far auto that shoots from wing line. */
  // TODO - retool after intake change
  // public static Command wing(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceA"))
        // ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceB")),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("NPieceC")),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake)
    // );
  // }

  /** Generalized 2+ piece auto. */
  // TODO - retool after intake change
  // private class AutoFactory extends SequentialCommandGroup {
    // public AutoFactory(Swerve drivetrain, Superstructure superstructure, Intake intake, String startPath, String... subsequentPaths) {
      // First shot
      // addCommands(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile(startPath))
        // ),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake)
      // );
      // for (String path : subsequentPaths) {
        // addCommands(
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile(path)),
          // new WaitCommand(0.7),
          // new ShooterAutomation(drivetrain, superstructure, intake)
        // );
      // }
    // }
  // }

  // public static Command center(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    // return new SequentialCommandGroup(
      // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.3),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("TwoPiece"))
        // ),
        // new WaitCommand(0.7),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new WaitUntilCommand(superstructure::atSetpoint),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("CenterB")),
        // new WaitCommand(0.7),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SPOOLING)),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("CenterC")),
        // new ShooterAutomation(drivetrain, superstructure, intake)
    // );
  // }

  /** Two piece amp side. */
  // public static Command twoPieceAmpSide(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(intake::atSetpoint),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(1),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(0.5),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideA"))
        // ),
        // new WaitCommand(1.5),
        // new ShooterAutomation(drivetrain, superstructure, intake),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideB")),
        // new WaitCommand(1),
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("AmpSideC")),
        // new ShooterAutomation(drivetrain, superstructure, intake)
    // );
  // }

  public static Command splineShoot(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new ParallelCommandGroup(
      new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("ShootTest"), false),
      new ShooterAutomation(drivetrain, superstructure, intake, true)
    );
  }

  /** Two piece side auto. */
  // public static Command twoPieceOpenSide(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    // return new SequentialCommandGroup(
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.IDLE);
          // intake.setState(IntakeState.MID);
          // PPLibTelemetry.setCurrentPath(PathPlannerPath.fromPathFile("TwoPiece"));
        // }),
        // new WaitUntilCommand(() -> Intake.pivot.getEncoder().getPosition() < 29),
        // new InstantCommand(() -> {
          // superstructure.setState(SuperstructureState.SPOOLING);
          // intake.setState(IntakeState.DOWN);
        // }),
        // new WaitCommand(1.5),
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        // new WaitCommand(1.5),
        // new ParallelCommandGroup(
          // new InstantCommand(() -> superstructure.setState(SuperstructureState.RECEIVE)),
          // new WaitCommand(1),
          // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("TwoPieceOpenSide"))
        // ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        // new WaitCommand(2),
        // new ParallelCommandGroup(
        // new FollowPathPlannerTrajectory(drivetrain, PathPlannerPath.fromPathFile("TwoPieceOpenSideBack")),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.PODIUM_READY),
        // ),
        // new InstantCommand(drivetrain::stop, drivetrain),
        // new ShooterAutomation(drivetrain, superstructure, intake)
        // new InstantCommand(() -> superstructure.setState(SuperstructureState.PODIUM_GO)),
        // new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    // );
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
