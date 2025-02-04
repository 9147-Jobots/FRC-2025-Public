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

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.logging.ElevatorIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(ElevatorConstants.FEED_FORWARD_VALUES[0][0], ElevatorConstants.FEED_FORWARD_VALUES[0][1]);
        io.configurePID(ElevatorConstants.PID_MODES[0][0],
                        ElevatorConstants.PID_MODES[0][1],
                        ElevatorConstants.PID_MODES[0][2]);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(ElevatorConstants.FEED_FORWARD_VALUES[1][0], ElevatorConstants.FEED_FORWARD_VALUES[1][1]);
        io.configurePID(ElevatorConstants.PID_MODES[1][0],
                        ElevatorConstants.PID_MODES[1][1],
                        ElevatorConstants.PID_MODES[1][2]);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(ElevatorConstants.FEED_FORWARD_VALUES[2][0], ElevatorConstants.FEED_FORWARD_VALUES[1][2]);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runPosition(double position) {
    io.setPosition(position, ffModel.calculate(position));

    // Log elevator setpoint
    Logger.recordOutput("Elevator/Setpoint", position);
  }

  /** Stops the elevator. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double[] getVelocityRPM() {
    return new double[] {Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec[0]),
            Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec[1])};
  }

  /** Returns the current velocity in radians per second. */
  public double[] getCharacterizationVelocity() {
    return new double[] {inputs.velocityRadPerSec[0], inputs.velocityRadPerSec[1]};
  }
}
