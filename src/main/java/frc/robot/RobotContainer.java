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

import frc.robot.Constants.PresetConstants;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.MechanismIOSparkMax;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Mechanism mechanism;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive = new Drive(new GyroIOPigeon2(true), 
                          new ModuleIOTalonFX(0),
                          new ModuleIOTalonFX(1),
                          new ModuleIOTalonFX(2),
                          new ModuleIOTalonFX(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive = new Drive(new GyroIO() {},
                          new ModuleIOSim(),
                          new ModuleIOSim(),
                          new ModuleIOSim(),
                          new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations

        drive = new Drive(new GyroIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {},
                          new ModuleIO() {});
        break;
    }

    elevator = new Elevator(new ElevatorIOSparkMax());
    mechanism = new Mechanism(new MechanismIOSparkMax());
    climber = new Climber(new ClimberIOSparkMax());

    // Set up auto routines
    NamedCommands.registerCommand("Test", new CoralIntake(mechanism));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());    

    // SmartDashboard.putData("auto", autoChooser);
    SmartDashboard.putData(autoChooser.getSendableChooser());

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
                                                        () -> -controller.getLeftY() * Constants.DriveConstants.X_IN,
                                                        () -> -controller.getLeftX() * Constants.DriveConstants.Y_IN,
                                                        () -> -controller.getRightX() * Constants.DriveConstants.OMEGA_IN));

    controller.start().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller2.x().onTrue(Commands.runOnce(() -> mechanism.coralRunVelocity(0)));
    controller2.a().onTrue(Commands.runOnce(() -> mechanism.pivotRunPosition(PresetConstants.PIVOT_REST)));
    controller2.b().onTrue(Commands.runOnce(() ->mechanism.pivotRunPosition(PresetConstants.PIVOT_Intake)));
    controller2.y().onTrue(Commands.runOnce(() -> mechanism.pivotRunPosition(PresetConstants.PIVOT_L4)));

    controller2.leftBumper().onTrue(Commands.runOnce(() -> mechanism.coralRunVelocity(-1000)));
    controller2.rightBumper().onTrue(new CoralIntake(mechanism));

    controller2.povUp().onTrue(Commands.runOnce(() -> elevator.runPosition(PresetConstants.ELEVATOR_L4)));
    controller2.povRight().onTrue(Commands.runOnce(() -> elevator.runPosition(PresetConstants.ELEVATOR_L3)));
    controller2.povDown().onTrue(Commands.runOnce(() -> elevator.runPosition(PresetConstants.ELEVATOR_L1_L2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
