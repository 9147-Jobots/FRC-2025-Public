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

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class DriveConstants {
    // Input constants
    public static final double X_IN = 0.25;
    public static final double Y_IN = 0.25;
    public static final double OMEGA_IN = 0.25;

    /** m/s */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
    public static final double TRACK_WIDTH_Y = 0.581;
    public static final double TRACK_WIDTH_X = 0.9135;
    public static final double DRIVE_BASE_RADIUS =
      Math.hypot(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED =  DriveConstants.MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);

  }

  public class VisionConstants {
    // Measurements for Transform3D
    public final class cameraFront { // 0.41 meters from the center backwards, 0.0 meters to the right, 0.2 meters up, 0.0 radians pitch, 0.0 radians roll, and pi radians/180 degreees yaw
        public static final double x = 0;
        public static final double y = 0;
        public static final double z = 0;
        public static final double pitch = 0;
        public static final double roll = 0;
        public static final double yaw = Math.PI;
    }
    public final class cameraBack {
        public static final double x = 0;
        public static final double y = 0;
        public static final double z = 0;
        public static final double pitch = 0;
        public static final double roll = 0;
        public static final double yaw = 0;
    }

    public final class PoseEstimatorConstants {
        public static final Matrix<N3, N1> stateStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{0.1, 0.1, 0.1});
    }
  }

  public class ModuleConstants {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double ODOMETRY_FREQUENCY = 250.0;
  }

  public class ModuleIODriveConstants {
    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    // MUST BE CALIBRATED
    public static final Rotation2d[] ABSOLUTE_ENCODER_OFFSETS = {Rotation2d.fromDegrees(-58.291406),
                                                                 Rotation2d.fromDegrees(33.176953),
                                                                 Rotation2d.fromDegrees(48.287109),
                                                                 Rotation2d.fromDegrees(84.678516)};
    
    public static final int[] driveTalonID = {0, 3, 6, 9};
    public static final int[] turnTalonID = {1, 4, 7, 10};
    public static final int[] CANcoderID = {2, 5, 8, 11};

    // MUST BE CALIBRATED
    public static final Boolean[] isTurnMotorInverted = {true, true, true, true};
    public static final Boolean[] isDriveMotorInverted = {true, false, false, true};

    public static final double TURN_SUPPLY_CURRENT_LIMIT = 30;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 40;

    public static final Boolean TURN_SUPPLY_CURRENT_ENABLE = true;
    public static final Boolean DRIVE_SUPPLY_CURRENT_ENABLE = true;

    public static final Boolean TURN_BRAKE_MODE = true;
    public static final Boolean DRIVE_BRAKE_MODE = true;
  }

  public class DriveCommandsConstants {
    public static final double DEADBAND = 0.1;
  }

  public class ElevatorConstants {
    public static final int[] CONTROLLER_ID = {12, 13};
    public static final MotorType[] MOTOR_TYPE = {MotorType.kBrushless, MotorType.kBrushless};

    public static final int[] CAN_TIMEOUT = {250, 250};
    public static final Boolean[] INVERTED = {false, true};
    public static final double[] VOLTAGE_COMPENSATION = {12.0, 12.0};
    public static final int[] SMART_CURRENT_LIMIT = {10, 10};
    public static final int MAX_VELOCITY = 5000;
    public static final int MAX_ACCELERATION = 5000;
    public static final float CONVERSION_FACTOR = 1;

    public static final double[][] PID_MODES = {{5, 0, 0}, // REAL
                                                {2, 0.5, 0}, // REPLAY
                                                {2, 0.5, 0}, // SIM
                                                {2, 0.5, 0}}; // DEFAULT

    public static final double[][] FEED_FORWARD_VALUES = {{1, 0, 0}, // REAL
                                                          {0.1, 0.05}, // REPLAY
                                                          {0.0, 0.03}, // SIM
                                                          {0, 0}}; // DEFAULT
  }

  public class ClimberConstants {
    public static final int[] CONTROLLER_ID = {12, 13};
    public static final MotorType[] MOTOR_TYPE = {MotorType.kBrushless, MotorType.kBrushless};

    public static final int[] CAN_TIMEOUT = {250, 250};
    public static final Boolean[] INVERTED = {false, true};
    public static final double[] VOLTAGE_COMPENSATION = {12.0, 12.0};
    public static final int[] SMART_CURRENT_LIMIT = {10, 10};
    public static final int MAX_VELOCITY = 5000;
    public static final int MAX_ACCELERATION = 5000;
    public static final float CONVERSION_FACTOR = 1;

    public static final double[][] PID_MODES = {{5, 0, 0}, // REAL
                                                {2, 0.5, 0}, // REPLAY
                                                {2, 0.5, 0}, // SIM
                                                {2, 0.5, 0}}; // DEFAULT

    public static final double[][] FEED_FORWARD_VALUES = {{1, 0, 0}, // REAL
                                                          {0.1, 0.05}, // REPLAY
                                                          {0.0, 0.03}, // SIM
                                                          {0, 0}}; // DEFAULT
  }

  public class MechanismConstants {
    public static final int[] CONTROLLER_ID = {0, 1, 2};
    public static final MotorType[] MOTOR_TYPE = {MotorType.kBrushless, MotorType.kBrushed, MotorType.kBrushed};

    public static final int CORAL_CAN_TIMEOUT = 250;
    public static final int ALGAE_CAN_TIMEOUT = 250;
    public static final int PIVOT_CAN_TIMEOUT = 250;

    public static final Boolean CORAL_INVERTED = false;
    public static final Boolean ALGAE_INVERTED = false;
    public static final Boolean PIVOT_INVERTED = false;

    public static final double CORAL_VOLTAGE_COMPENSATION = 12.0;
    public static final double ALGAE_VOLTAGE_COMPENSATION = 12.0;
    public static final double PIVOT_VOLTAGE_COMPENSATION = 12.0;

    public static final int CORAL_SMART_CURRENT_LIMIT = 30;
    public static final int ALGAE_SMART_CURRENT_LIMIT = 30;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 30;

    public static final double[][] CORAL_PID_MODES = {{1, 0, 0}, // REAL
                                                      {1, 0, 0}, // REPLAY
                                                      {0.5, 0, 0}, // SIM
                                                      {0.5, 0, 0}}; // DEFAULT
    
    public static final double[][] ALGAE_PID_MODES = {{1, 0, 0}, // REAL
                                                      {1, 0, 0}, // REPLAY
                                                      {0.5, 0, 0}, // SIM
                                                      {0.5, 0, 0}}; // DEFAULT

    public static final double[][] PIVOT_PID_MODES = {{1, 0, 0}, // REAL
                                                      {1, 0, 0}, // REPLAY
                                                      {0.5, 0, 0}, // SIM
                                                      {1, 0, 0}}; // DEFAULT

    public static final double[][] FEED_FORWARD_VALUES = {{1, 0, 0}, // REAL
                                                          {0.1, 0.05}, // REPLAY
                                                          {0.0, 0.03}, // SIM
                                                          {0, 0}}; // DEFAULT
  }
}
