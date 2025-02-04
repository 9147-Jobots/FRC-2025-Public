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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.logging.MechanismIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Mechanism extends SubsystemBase {
  private final MechanismIO io;
  private final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine algaeSysId;
  private final SysIdRoutine coralSysId;
  private final SysIdRoutine pivotSysId;

  /** Creates a new Mechansim. */
  public Mechanism(MechanismIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(MechanismConstants.FEED_FORWARD_VALUES[0][0], 
                                             MechanismConstants.FEED_FORWARD_VALUES[0][1]);
        io.algaeConfigurePID(MechanismConstants.ALGAE_PID_MODES[0][0],
                            MechanismConstants.ALGAE_PID_MODES[0][1],
                            MechanismConstants.ALGAE_PID_MODES[0][2]);
        
        io.coralConfigurePID(MechanismConstants.CORAL_PID_MODES[0][0],
                            MechanismConstants.CORAL_PID_MODES[0][1],
                            MechanismConstants.CORAL_PID_MODES[0][2]);

        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(MechanismConstants.FEED_FORWARD_VALUES[1][0], 
                                             MechanismConstants.FEED_FORWARD_VALUES[1][1]);

        io.algaeConfigurePID(MechanismConstants.ALGAE_PID_MODES[1][0],
                             MechanismConstants.ALGAE_PID_MODES[1][1],
                             MechanismConstants.ALGAE_PID_MODES[1][2]);
                         
        io.coralConfigurePID(MechanismConstants.CORAL_PID_MODES[1][0],
                             MechanismConstants.CORAL_PID_MODES[1][1],
                             MechanismConstants.CORAL_PID_MODES[1][2]);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(MechanismConstants.FEED_FORWARD_VALUES[2][0], 
                                             MechanismConstants.FEED_FORWARD_VALUES[1][2]);
        break;
    }

    // Configure SysId
    algaeSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Mechanism/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runAlgaeVolts(voltage.in(Volts)), null, this));
    coralSysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Mechanism/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism((voltage) -> runCoralVolts(voltage.in(Volts)), null, this));
    pivotSysId =
            new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Mechanism/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism((voltage) -> runPivotVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Mechansim", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runAlgaeVolts(double volts) {
    io.algaeSetVoltage(volts);
  }

  /** Run open loop at the specified voltage. */
  public void runCoralVolts(double volts) {
    io.coralSetVoltage(volts);
  }

  /** Run open loop at the specified voltage. */
  public void runPivotVolts(double volts) {
    io.pivotSetVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void algaeRunPosition(double position) {
    io.algaeSetPosition(position, ffModel.calculate(position));

    // Log mechansim setpoint
    Logger.recordOutput("Mechansim/AlgaeSetpoint", position);
  }

  /** Run closed loop at the specified velocity. */
  public void coralRunPosition(double position) {
    io.coralSetPosition(position, ffModel.calculate(position));

    // Log mechansim setpoint
    Logger.recordOutput("Mechansim/CoralSetpoint", position);
  }

  /** Run closed loop at the specified velocity. */
  public void pivotRunPosition(double position) {
    io.pivotSetPosition(position, ffModel.calculate(position));

    // Log mechansim setpoint
    Logger.recordOutput("Mechansim/PivotSetpoint", position);
  }

  /** Stops the mechansim. */
  public void coralStop() {
    io.coralStop();
  }

  /** Stops the mechansim. */
  public void algaeStop() {
    io.algaeStop();
  }

  /** Stops the mechansim. */
  public void pivotStop() {
    io.pivotStop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command algaeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return algaeSysId.quasistatic(direction);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command coralSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return coralSysId.quasistatic(direction);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command coralSysIdDynamic(SysIdRoutine.Direction direction) {
    return coralSysId.dynamic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command algaeSysIdDynamic(SysIdRoutine.Direction direction) {
    return algaeSysId.dynamic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getCoralVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.coralVelocityRadPerSec);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getAlgaeVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.algaeVelocityRadPerSec);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getPivotVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.pivotVelocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCoralCharacterizationVelocity() {
    return inputs.coralVelocityRadPerSec;
  }

  /** Returns the current velocity in radians per second. */
  public double getAlgaeCharacterizationVelocity() {
    return inputs.algaeVelocityRadPerSec;
  }

  /** Returns the current velocity in radians per second. */
  public double getPivotCharacterizationVelocity() {
    return inputs.pivotVelocityRadPerSec;
  }
}
