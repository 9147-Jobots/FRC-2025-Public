// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.Mechanism;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  /** Creates a new CoralIntake. */
  Mechanism m_mechanism;

  boolean coralContacted = false;

  double endTime;
  double startTime;

  final double VELOCITY = 2000;
  final double MAX_DURATION = 10;
  final double TIME_AFTER_CORAL_CONTACTED = 1;
  final double MAX_CURRENT = 15;
  final double CURRENT_BUFFER = 0.1;

  public CoralIntake(Mechanism mechanism) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mechanism);

    m_mechanism = mechanism;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_mechanism.coralRunVelocity(VELOCITY);
    coralContacted = false;
    startTime = Timer.getFPGATimestamp();
    endTime = startTime + MAX_DURATION; // move to CONSTANTS
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Coral current", m_mechanism.getCoralCurrent());

    if (Timer.getFPGATimestamp() - startTime > CURRENT_BUFFER) {
      if (m_mechanism.getCoralCurrent() > MAX_CURRENT) {
        coralContacted = true;
      }
    }

    if (coralContacted) {
      if (Timer.getFPGATimestamp() + TIME_AFTER_CORAL_CONTACTED < endTime) {
        endTime = Timer.getFPGATimestamp() + TIME_AFTER_CORAL_CONTACTED;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_mechanism.coralStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() >= endTime) {
      return true;
    }
    return false;
  }
}
