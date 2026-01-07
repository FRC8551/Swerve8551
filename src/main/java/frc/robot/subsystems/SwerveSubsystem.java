// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerve;

  public SwerveSubsystem() {

    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
        Meter.of(4)),
        Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(Meter.of(16),
            Meter.of(4)),
            Rotation2d.fromDegrees(0));

    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(Units.feetToMeters(16), startingPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Swerve configuration
    swerve.setAngularVelocityCompensation(false, false, 0);
    swerve.setAutoCenteringModules(false);
    swerve.setChassisDiscretization(true, 0.02); // Change this to true
    swerve.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation
    swerve.setHeadingCorrection(false);
    swerve.setModuleEncoderAutoSynchronize(true, 1);
    swerve.setModuleStateOptimization(true);
    swerve.setMotorIdleMode(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Module Offset FL",
        Math.round(swerve.getModules()[0].getAbsolutePosition() * 1000) / 1000);
    SmartDashboard.putNumber("Module Offset FR",
        Math.round(swerve.getModules()[1].getAbsolutePosition() * 1000) / 1000);
    SmartDashboard.putNumber("Module Offset BL",
        Math.round(swerve.getModules()[2].getAbsolutePosition() * 1000) / 1000);
    SmartDashboard.putNumber("Module Offset BR",
        Math.round(swerve.getModules()[3].getAbsolutePosition() * 1000) / 1000);

  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerve.drive(velocity.get());
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerve.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDrive getSwerveDrive() {
    return swerve;
  }
}
