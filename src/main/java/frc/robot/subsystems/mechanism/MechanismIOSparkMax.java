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

import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;

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
public class MechanismIOSparkMax implements MechanismIO {
  private final SparkMax coral = new SparkMax(MechanismConstants.CORAL_ID, MechanismConstants.CORAL_TYPE);
  private final SparkMax algae = new SparkMax(MechanismConstants.ALGAE_ID, MechanismConstants.ALGAE_TYPE);
  private final SparkMax pivot = new SparkMax(MechanismConstants.PIVOT_ID, MechanismConstants.PIVOT_TYPE);

  private final SparkMaxConfig coral_config = new SparkMaxConfig();
  private final SparkMaxConfig algae_config = new SparkMaxConfig();
  private final SparkMaxConfig pivot_config = new SparkMaxConfig();

  private final RelativeEncoder coral_encoder = coral.getEncoder();
  private final RelativeEncoder algae_encoder = algae.getEncoder();
  private final RelativeEncoder pivot_encoder = pivot.getEncoder();

  private final SparkClosedLoopController coral_pid = coral.getClosedLoopController();
  private final SparkClosedLoopController algae_pid = algae.getClosedLoopController();
  private final SparkClosedLoopController pivot_pid = pivot.getClosedLoopController();
  
  public MechanismIOSparkMax() {
    int pid_index;

    switch (Constants.currentMode) {
      case REAL:
        pid_index = 0;
        break;

      case REPLAY:
        pid_index = 1;
        break;

      case SIM:
        pid_index = 2;
        break;

      default:
        pid_index = 3;
        break;
    }

    coral.setCANTimeout(MechanismConstants.CORAL_CAN_TIMEOUT);
    coral_config.inverted(MechanismConstants.CORAL_INVERTED)
                .voltageCompensation(MechanismConstants.CORAL_VOLTAGE_COMPENSATION)
                .smartCurrentLimit(MechanismConstants.CORAL_SMART_CURRENT_LIMIT)
                .closedLoop.pid(MechanismConstants.CORAL_PID_MODES[pid_index][0],
                                MechanismConstants.CORAL_PID_MODES[pid_index][1],
                                MechanismConstants.CORAL_PID_MODES[pid_index][2]);
    coral.configure(coral_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algae.setCANTimeout(MechanismConstants.ALGAE_CAN_TIMEOUT);
    algae_config.inverted(MechanismConstants.ALGAE_INVERTED)
                .voltageCompensation(MechanismConstants.ALGAE_VOLTAGE_COMPENSATION)
                .smartCurrentLimit(MechanismConstants.ALGAE_SMART_CURRENT_LIMIT)
                .closedLoop.pid(MechanismConstants.ALGAE_PID_MODES[pid_index][0],
                                MechanismConstants.ALGAE_PID_MODES[pid_index][1],
                                MechanismConstants.ALGAE_PID_MODES[pid_index][2]);
    algae.configure(algae_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivot.setCANTimeout(MechanismConstants.PIVOT_CAN_TIMEOUT);
    pivot_config.inverted(MechanismConstants.PIVOT_INVERTED)
                .voltageCompensation(MechanismConstants.PIVOT_VOLTAGE_COMPENSATION)
                .smartCurrentLimit(MechanismConstants.PIVOT_SMART_CURRENT_LIMIT)
                .closedLoop.pid(MechanismConstants.PIVOT_PID_MODES[pid_index][0],
                                MechanismConstants.PIVOT_PID_MODES[pid_index][1],
                                MechanismConstants.PIVOT_PID_MODES[pid_index][2]);
    pivot.configure(pivot_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {
    inputs.coralPositionRad = Units.rotationsToRadians(coral_encoder.getPosition());
    inputs.algaePositionRad = Units.rotationsToRadians(algae_encoder.getPosition());
    inputs.pivotPositionRad = Units.rotationsToRadians(pivot_encoder.getPosition());

    inputs.coralVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(coral_encoder.getVelocity());
    inputs.algaeVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(algae_encoder.getVelocity());
    inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivot_encoder.getVelocity());

    inputs.coralAppliedVolts = coral.getAppliedOutput() * coral.getBusVoltage();
    inputs.algaeAppliedVolts = algae.getAppliedOutput() * algae.getBusVoltage();
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();

    inputs.coralCurrentAmps = coral.getOutputCurrent();
    inputs.algaeCurrentAmps = algae.getOutputCurrent();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();
  }

  @Override
  public void coralSetVoltage(double volts) {
    coral.setVoltage(volts);
  }

  @Override
  public void algaeSetVoltage(double volts) {
    algae.setVoltage(volts);
  }

  @Override
  public void pivotSetVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  @Override
  public void coralSetVelocity(double targetPosition, double ffVolts) {
    coral_pid.setReference(
        targetPosition,
        ControlType.kVelocity);
  }

  @Override
  public void algaeSetVelocity(double targetPosition, double ffVolts) {
    algae_pid.setReference(
        targetPosition,
        ControlType.kVelocity);
  }

  @Override
  public void coralSetPosition(double targetPosition, double ffVolts) {
    coral_pid.setReference(
        targetPosition,
        ControlType.kPosition);
  }

  @Override
  public void algaeSetPosition(double targetPosition, double ffVolts) {
    algae_pid.setReference(
        targetPosition,
        ControlType.kPosition);
  }

  @Override
  public void pivotSetPosition(double targetPosition, double ffVolts) {
    pivot_pid.setReference(
        targetPosition,
        ControlType.kPosition);
  }

  @Override
  public void coralStop() {
    coral.stopMotor();
  }

  @Override
  public void algaeStop() {
    algae.stopMotor();
  }

  @Override
  public void pivotStop() {
    pivot.stopMotor();
  }
}
