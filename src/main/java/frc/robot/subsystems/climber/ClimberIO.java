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

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double[] positionRad = {0, 0};
    public double[] velocityRadPerSec = {0, 0};
    public double[] appliedVolts = {0, 0};
    public double[] currentAmps = {0, 0};
  }


  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setPosition(double targetPosition, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Periodic code */
  public default void periodic() {}
}
