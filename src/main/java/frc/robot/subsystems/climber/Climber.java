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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.logging.ClimberIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Elevator. */
  public Climber(ClimberIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(ClimberConstants.FEED_FORWARD_VALUES[0][0], 
                                             ClimberConstants.FEED_FORWARD_VALUES[0][1]);
        break;

      case REPLAY:
        ffModel = new SimpleMotorFeedforward(ClimberConstants.FEED_FORWARD_VALUES[1][0], 
                                             ClimberConstants.FEED_FORWARD_VALUES[1][1]);
        break;

      case SIM:
        ffModel = new SimpleMotorFeedforward(ClimberConstants.FEED_FORWARD_VALUES[2][0], 
                                             ClimberConstants.FEED_FORWARD_VALUES[2][1]);
        break;
        
      default:
        ffModel = new SimpleMotorFeedforward(ClimberConstants.FEED_FORWARD_VALUES[3][0], 
                                             ClimberConstants.FEED_FORWARD_VALUES[3][1]);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    io.periodic();
  }

  /** Run closed loop at the specified velocity. */
  public void runPosition(double position) {
    io.setPosition(position, ffModel.calculate(position));

    // Log climber setpoint
    Logger.recordOutput("Climber/Setpoint", position);
  }

  /** Stops the climber. */
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
