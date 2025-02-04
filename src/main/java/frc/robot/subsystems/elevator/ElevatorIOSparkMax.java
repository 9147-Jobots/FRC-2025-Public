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

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax[] controllers = {new SparkMax(ElevatorConstants.CONTROLLER_ID[0], ElevatorConstants.MOTOR_TYPE[0]),
                                          new SparkMax(ElevatorConstants.CONTROLLER_ID[1], ElevatorConstants.MOTOR_TYPE[1])};

  // MUST have same length as controllers
  private final SparkMaxConfig[] configs = {new SparkMaxConfig(),
                                            new SparkMaxConfig()};

  private final RelativeEncoder[] encoders = {controllers[0].getEncoder(), 
                                              controllers[1].getEncoder()};

  private final SparkClosedLoopController[] pids = {controllers[0].getClosedLoopController(),
                                                    controllers[1].getClosedLoopController()};

  public ElevatorIOSparkMax() {
    for (int index = 0; index < controllers.length; index++) {
      controllers[index].setCANTimeout(ElevatorConstants.CAN_TIMEOUT[index]);
      configs[index].inverted(ElevatorConstants.INVERTED[index]);
      configs[index].voltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION[index]);
      configs[index].smartCurrentLimit(ElevatorConstants.SMART_CURRENT_LIMIT[index]);
      controllers[index].configure(configs[index], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRad = new double[] {Units.rotationsToRadians(encoders[0].getPosition()), 
                                       Units.rotationsToRadians(encoders[1].getPosition())};

    inputs.velocityRadPerSec = new double[] {Units.rotationsPerMinuteToRadiansPerSecond(encoders[0].getVelocity()), 
                                             Units.rotationsPerMinuteToRadiansPerSecond(encoders[1].getVelocity())};

    inputs.appliedVolts = new double[] {controllers[0].getAppliedOutput() * controllers[0].getBusVoltage(),
                                        controllers[1].getAppliedOutput() * controllers[1].getBusVoltage()};

    inputs.currentAmps = new double[] {controllers[0].getOutputCurrent(), 
                                       controllers[1].getOutputCurrent()};
  }

  @Override
  public void setPosition(double targetPosition, double ffVolts) {
    pids[0].setReference(
        targetPosition,
        ControlType.kPosition);
  }

  @Override
  public void stop() {
    controllers[0].stopMotor();
    controllers[1].stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    configs[0].closedLoop.pidf(kP, kI, kD, 0);
    configs[1].closedLoop.pidf(kP, kI, kD, 0);
  }
}
