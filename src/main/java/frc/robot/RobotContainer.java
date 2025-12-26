// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Swerve Input Streams
  private final SwerveInputStream robotRelativeAngularVelocity = SwerveInputStream
      .of(swerveSubsystem.getSwerveDrive(), () -> -driverController.getLeftY(), () -> -driverController.getLeftX())
      .withControllerRotationAxis(() -> -driverController.getRightX())
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream allianceRelativeAngularVelocity = robotRelativeAngularVelocity.copy()
      .allianceRelativeControl(true);

  private final SwerveInputStream allianceRelativeDirectAngle = allianceRelativeAngularVelocity.copy()
      .withControllerHeadingAxis(driverController::getRightX, driverController::getRightY)
      .headingWhile(true);

  // Commands
  private final Command driveRobotOrientedAngularVelocity = swerveSubsystem
      .driveRobotOriented(robotRelativeAngularVelocity);

  private final Command driveFieldOrientedAngularVelocity = swerveSubsystem
      .driveFieldOriented(allianceRelativeAngularVelocity);

  private final Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(allianceRelativeDirectAngle);

  public RobotContainer() {
    configureBindings();

    // Set default subsystem commands
    swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
