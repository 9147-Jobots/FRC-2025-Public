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

package frc.robot.subsystems.drive;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleIODriveConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax driveSparkMax;
  private final SparkMaxConfig driveSparkConfig = new SparkMaxConfig();

  private final SparkMax turnSparkMax;
  private final SparkMaxConfig turnSparkConfig = new SparkMaxConfig();

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new SparkMax(1, MotorType.kBrushless);
        turnSparkMax = new SparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new SparkMax(3, MotorType.kBrushless);
        turnSparkMax = new SparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new SparkMax(5, MotorType.kBrushless);
        turnSparkMax = new SparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new SparkMax(7, MotorType.kBrushless);
        turnSparkMax = new SparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.configure(driveSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(turnSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkConfig.inverted(isTurnMotorInverted);
    turnSparkConfig.smartCurrentLimit(40);
    turnSparkConfig.voltageCompensation(12.0);
    turnSparkConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / ModuleConstants.ODOMETRY_FREQUENCY));

    driveSparkConfig.smartCurrentLimit(30);
    driveSparkConfig.voltageCompensation(12.0);
    driveSparkConfig.signals.primaryEncoderVelocityPeriodMs((int) (1000.0 / ModuleConstants.ODOMETRY_FREQUENCY));

    turnSparkMax.configure(turnSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveSparkMax.configure(driveSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder.setPosition(0.0);
    turnRelativeEncoder.setPosition(0.0);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / ModuleIODriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / ModuleIODriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / ModuleIODriveConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / ModuleIODriveConstants.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / ModuleIODriveConstants.DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / ModuleIODriveConstants.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(driveSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(turnSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
