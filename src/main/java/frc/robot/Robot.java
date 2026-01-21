// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  enum DriveMode {
    RobotOriented,
    FieldOrientedAngularVelocity,
    FieldOrientedDirectAngle
  }

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final SendableChooser<DriveMode> m_driveModeChooser = new SendableChooser<>();
  private DriveMode m_lastDriveMode;

  public Robot() {
    m_robotContainer = new RobotContainer();

    m_driveModeChooser.setDefaultOption("Field-Oriented Direct Angle", DriveMode.FieldOrientedDirectAngle);
    m_driveModeChooser.addOption("Field-Oriented Angular Velocity", DriveMode.FieldOrientedAngularVelocity);
    m_driveModeChooser.addOption("Robot-Oriented", DriveMode.RobotOriented);

    SmartDashboard.putData("Drive Mode", m_driveModeChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (m_driveModeChooser.getSelected() != m_lastDriveMode) {
      m_robotContainer.changeDriveMode(m_driveModeChooser.getSelected());
      m_lastDriveMode = m_driveModeChooser.getSelected();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
