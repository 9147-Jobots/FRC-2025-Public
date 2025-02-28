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

package frc.robot.subsystems.mechanism;

import org.littletonrobotics.junction.AutoLog;

public interface MechanismIO {
  @AutoLog
  public static class MechanismIOInputs {
    public double coralPositionRad = 0;
    public double algaePositionRad = 0;
    public double pivotPositionRad = 0;

    public double coralVelocityRadPerSec = 0;
    public double algaeVelocityRadPerSec = 0;
    public double pivotVelocityRadPerSec = 0;

    public double coralAppliedVolts = 0;
    public double algaeAppliedVolts = 0;
    public double pivotAppliedVolts = 0;

    public double coralCurrentAmps = 0;
    public double algaeCurrentAmps = 0;
    public double pivotCurrentAmps = 0;

  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(MechanismIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void coralSetVoltage(double volts) {}

  /** Run open loop at the specified voltage. */
  public default void algaeSetVoltage(double volts) {}

  /** Run open loop at the specified voltage. */
  public default void pivotSetVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void coralSetVelocity(double targetPosition, double ffVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void algaeSetVelocity(double targetPosition, double ffVolts) {}

  /** Run closed loop at the specified position. */
  public default void coralSetPosition(double targetPosition, double ffVolts) {}

  /** Run closed loop at the specified position. */
  public default void algaeSetPosition(double targetPosition, double ffVolts) {}

  /** Run closed loop at the specified position. */
  public default void pivotSetPosition(double targetPosition, double ffVolts) {}

  /** Stop in open loop. */
  public default void coralStop() {}

  /** Stop in open loop. */
  public default void algaeStop() {}

  /** Stop in open loop. */
  public default void pivotStop() {}

  /** gets the output current of the coral motor */
  public default double getCoralMotorCurrent() { return 0; }

  public default void periodic() {}
}
