// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.SuperstructureDefault.StateChangeCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /** One piece auto. */
  public static Command onePiece(Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new StateChangeCommand(superstructure, intake, SuperstructureState.SPOOLING),
        new WaitCommand(1.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(1.5),
        new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE)
    );
  }

  /** Two piece auto (no splines). */
  public static Command twoPiece(Swerve drivetrain, Superstructure superstructure, Intake intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> { superstructure.setState(SuperstructureState.IDLE); }),
        new StateChangeCommand(superstructure, intake, SuperstructureState.SPOOLING),
        new WaitCommand(1.5),
        new InstantCommand(() -> superstructure.setState(SuperstructureState.SHOOTING)),
        new WaitCommand(1.5),
        new StateChangeCommand(superstructure, intake, SuperstructureState.RECEIVE),
        // TODO - why doesn't this move?
        new ParallelRaceGroup(
          new InstantCommand(() -> {
            drivetrain.drive(new ChassisSpeeds(-0.5, 0, 0)); }).repeatedly(),
          new WaitCommand(1)
        ),
        new InstantCommand(drivetrain::stop)
        // new InstantCommand(() -> { intake.setState(IntakeState.DOWN); })
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
