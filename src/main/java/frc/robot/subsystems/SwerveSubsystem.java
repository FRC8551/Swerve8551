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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
    swerve.setAutoCenteringModules(true);
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

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerve.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public SwerveDrive getSwerveDrive() {
    return swerve;
  }
}
