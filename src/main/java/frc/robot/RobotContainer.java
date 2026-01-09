// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.DriveMode;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Swerve Input Streams
  private final SwerveInputStream m_robotRelative = SwerveInputStream
      .of(m_swerveSubsystem.getSwerveDrive(), () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream m_allianceRelativeAngularVelocity = m_robotRelative.copy()
      .allianceRelativeControl(true);

  private final SwerveInputStream m_allianceRelativeDirectAngle = m_allianceRelativeAngularVelocity.copy()
      .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
      .headingWhile(true);

  // Commands
  private final Command m_driveRobotOrientedAngularVelocity = m_swerveSubsystem
      .driveRobotOriented(m_robotRelative);

  private final Command m_driveFieldOrientedAngularVelocity = m_swerveSubsystem
      .driveFieldOriented(m_allianceRelativeAngularVelocity);

  private final Command m_driveFieldOrientedDirectAngle = m_swerveSubsystem
      .driveFieldOriented(m_allianceRelativeDirectAngle);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public void changeDriveMode(DriveMode driveMode) {
    if (m_swerveSubsystem.getCurrentCommand() != null) {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

    switch (driveMode) {
      case RobotOriented:
        m_swerveSubsystem.setDefaultCommand(m_driveRobotOrientedAngularVelocity);
        break;
      case FieldOrientedAngularVelocity:
        m_swerveSubsystem.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
        break;
      case FieldOrientedDirectAngle:
        m_swerveSubsystem.setDefaultCommand(m_driveFieldOrientedDirectAngle);
      default:
        break;
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
