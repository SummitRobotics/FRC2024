package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.Functions;

public class Shoot extends Command {

  private final Translation2d speakerPose;;
  private Swerve drivetrain;
  // private Superstructure superstructure;
  // private Intake intake;
  private PIDController pid = new PIDController(1, 0, 0.01);

  public Shoot(Swerve drivetrain/*, Superstructure superstructure, Intake intake*/) {
    speakerPose = DriverStation.getAlliance().get() == Alliance.Blue ? new Translation2d(0, 5) : new Translation2d(16.5, 5.5);
    this.drivetrain = drivetrain;
    // this.superstructure = superstructure;
    // this.intake = intake;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Align to speaker
    drivetrain.drive(new ChassisSpeeds(0, 0, -pid.calculate(drivetrain.getPose().getRotation().getRadians(),
      Functions.clampRadians(Math.PI + Math.atan((speakerPose.getY() - drivetrain.getPose().getY()) / (speakerPose.getX() - drivetrain.getPose().getX())))
    )));
  }
}
