// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;

  public SwerveSubsystem() {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(Units.feetToMeters(16));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Swerve configuration
    m_swerve.setAngularVelocityCompensation(false, false, 0);
    m_swerve.setAutoCenteringModules(false);
    m_swerve.setChassisDiscretization(true, 0.02); // Change this to true
    m_swerve.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation
    m_swerve.setHeadingCorrection(false);
    m_swerve.setModuleEncoderAutoSynchronize(true, 1);
    m_swerve.setModuleStateOptimization(true);
    m_swerve.setMotorIdleMode(false);

    setupPathPlanner();
  }

  @Override
  public void periodic() {
    ChassisSpeeds robotVelocity = m_swerve.getRobotVelocity();
    double velocity = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) * 2.23694;
    SmartDashboard.putNumber(SwerveConstants.kSlash + "Speedometer (MPH)", Math.round(velocity * 1000.0) / 1000.0);
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          m_swerve::getPose,
          m_swerve::resetOdometry,
          m_swerve::getRobotVelocity,
          (speeds, feedforwards) -> m_swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0),
              new PIDConstants(5, 0, 0)),
          config,
          this::isRedAlliance,
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_swerve.field.getObject("path").setPoses(poses);
    });
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerve.drive(velocity.get());
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerve.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerve;
  }
}
