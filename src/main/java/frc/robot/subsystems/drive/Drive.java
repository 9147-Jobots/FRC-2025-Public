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

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import lib.util.LocalADStarAK;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    // SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = new RobotConfig(0d, 0d, new ModuleConfig(DriveConstants.TRACK_WIDTH_Y, 
                                                                   DriveConstants.TRACK_WIDTH_X, 
                                                                   DriveConstants.MAX_LINEAR_SPEED, 
                                                                   DCMotor.getKrakenX60(4), 
                                                                   DriveConstants.MAX_ANGULAR_SPEED, 
                                                                   DriveConstants.DRIVE_BASE_RADIUS, 
                                                                   8), getModuleTranslations());
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getCurrentSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              DriveConstants.TRANSLATION_PID_CONSTANTS,
              DriveConstants.ROTATION_PID_CONSTANTS
            ),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this); /** Reference to this subsystem to set requirements */
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {Logger.recordOutput("Odometry/Trajectory", 
                                                                 activePath.toArray(new Pose2d[activePath.size()]));
                                                                });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                                                                });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(null,
                                    null,
                                    null,
                                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> {for (int i = 0; i < 4; i++) {modules[i].runCharacterization(voltage.in(Volts));
                                                                                 }
                                                    }, null, this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);

    var visionFieldPose = Vision.GetFieldPose();
    addVisionMeasurement(visionFieldPose.estimatedPose.toPose2d(), visionFieldPose.timestampSeconds);

    for (var module : modules) {
      module.updateInputs();

    }

    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    SmartDashboard.putNumber("Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Rotation Gyro", gyroInputs.yawPosition.getDegrees());
    SmartDashboard.putNumber("Rotations", gyroInputs.yawPosition.getDegrees() / 360);

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Module " + i + " angle", modules[i].getAngle().getDegrees());
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        var angle = gyroInputs.odometryYawPositions[i].getDegrees();
        rawGyroRotation = Rotation2d.fromDegrees(angle%360);
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      Module module = modules[i];
      setpointStates[i].optimize(module.getAngle());
      SwerveModuleState moduleState = setpointStates[i];
      // new function created to manually set the setpoint and angle of each individual module state
      module.setState(moduleState);

      SwerveModuleState state = module.getState();
      optimizedSetpointStates[i] = state;
      SmartDashboard.putNumber("Module " + i + " speed", state.speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + i + " speed", state.angle.getDegrees());
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current robot-relative speeds */
  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }
}
