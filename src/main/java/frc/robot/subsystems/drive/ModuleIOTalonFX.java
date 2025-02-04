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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFXConfiguration driveConfig;
  private final TalonFX turnTalon;
  private final TalonFXConfiguration turnConfig;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private boolean isTurnMotorInverted;
  private boolean isDriveMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    if (!(0 <= index && index <= 3)) {
      throw new RuntimeException("Invalid module index");
    }

    driveConfig = new TalonFXConfiguration();
    turnConfig = new TalonFXConfiguration();
    driveTalon = new TalonFX(ModuleIODriveConstants.driveTalonID[index]);
    turnTalon = new TalonFX(ModuleIODriveConstants.turnTalonID[index]);
    cancoder = new CANcoder(ModuleIODriveConstants.CANcoderID[index]);
    absoluteEncoderOffset = ModuleIODriveConstants.ABSOLUTE_ENCODER_OFFSETS[index];
    isTurnMotorInverted = ModuleIODriveConstants.isTurnMotorInverted[index];
    isDriveMotorInverted = ModuleIODriveConstants.isDriveMotorInverted[index];
      
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleIODriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = ModuleIODriveConstants.DRIVE_SUPPLY_CURRENT_ENABLE;
    driveConfig.MotorOutput.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(ModuleIODriveConstants.DRIVE_BRAKE_MODE);
      
    turnConfig.CurrentLimits.SupplyCurrentLimit = ModuleIODriveConstants.TURN_SUPPLY_CURRENT_LIMIT;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = ModuleIODriveConstants.TURN_SUPPLY_CURRENT_ENABLE;
    turnConfig.MotorOutput.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turnTalon.getConfigurator().apply(turnConfig);

    setTurnBrakeMode(ModuleIODriveConstants.TURN_BRAKE_MODE);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / ModuleIODriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / ModuleIODriveConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / ModuleIODriveConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / ModuleIODriveConstants.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / ModuleIODriveConstants.DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / ModuleIODriveConstants.TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
