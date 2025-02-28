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

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax[] controllers = {new SparkMax(ClimberConstants.CONTROLLER_ID[0], ClimberConstants.MOTOR_TYPE[0]),
                                          new SparkMax(ClimberConstants.CONTROLLER_ID[1], ClimberConstants.MOTOR_TYPE[1])};

  // MUST have same length as controllers
  private final SparkMaxConfig[] configs = {new SparkMaxConfig(),
                                            new SparkMaxConfig()};

  private final RelativeEncoder[] encoders = {controllers[0].getEncoder(), 
                                              controllers[1].getEncoder()};

  private final SparkClosedLoopController[] pids = {controllers[0].getClosedLoopController(),
                                                    controllers[1].getClosedLoopController()};

  public ClimberIOSparkMax() {
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

    for (int index = 0; index < controllers.length; index++) {
      controllers[index].setCANTimeout(ClimberConstants.CAN_TIMEOUT[index]);
      
      configs[index]
        .inverted(ClimberConstants.INVERTED[index])
        .voltageCompensation(ClimberConstants.VOLTAGE_COMPENSATION[index])
        .smartCurrentLimit(ClimberConstants.SMART_CURRENT_LIMIT[index])
        .encoder.positionConversionFactor(ClimberConstants.CONVERSION_FACTOR);
      
      configs[index].closedLoop
                         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                         .pid(ClimberConstants.PID_MODES[pid_index][0],
                              ClimberConstants.PID_MODES[pid_index][1],
                              ClimberConstants.PID_MODES[pid_index][2])
                         .maxMotion.maxVelocity(ClimberConstants.MAX_VELOCITY)
                                   .maxAcceleration(ClimberConstants.MAX_ACCELERATION)
                                   .allowedClosedLoopError(1);
      

      controllers[index].configure(configs[index], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
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
    pids[0].setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    pids[1].setReference(targetPosition, ControlType.kMAXMotionPositionControl);
    SmartDashboard.putNumber("Climber set point", targetPosition);
  }

  @Override
  public void stop() {
    controllers[0].stopMotor();
    controllers[1].stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimberMotor1Value", encoders[0].getPosition());
    SmartDashboard.putNumber("ClimberMotor2Value", encoders[1].getPosition());
  }
}
