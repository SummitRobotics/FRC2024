// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve_new;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** PID values and physical constants for SDS Mk4i modules. */
  public static enum MODULE_PRESET {
    SDS_MK4i_L1(8.14, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.0, 0.0, 0.0}, 0.533333333333),
    SDS_MK4i_L2(6.75, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.25, 0, 0.005}, 0.533333333333),
    SDS_MK4i_L3(6.12, 150.0 / 7.0, Units.inchesToMeters(4),
        new double[]{0.25, 0.0, 0.005}, 0.533333333333);

    public final double driveGearRatio;
    public final double turnGearRatio;
    public final double wheelDiameter;
    public final double[] turnPID;
    public final double turnToDrive;

    MODULE_PRESET(double driveGearRatio, double turnGearRatio, double wheelDiameter,
        double[] turnPID, double turnToDrive) {
      this.driveGearRatio = driveGearRatio;
      this.turnGearRatio = turnGearRatio;
      this.wheelDiameter = wheelDiameter;
      this.turnPID = turnPID;
      this.turnToDrive = turnToDrive;
    }
  }
}
