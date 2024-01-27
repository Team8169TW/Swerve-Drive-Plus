// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Input and Output
  public static final class IOConstants {

    public static final double kDeadband = 0.1;

    public static final int kControllerDriver = 1;
    public static final int kControllerOperator = 0;
  }

  // Swerve modules
  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1.0 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / (150/7.0);
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // Used in working code currently
    public static final double kPTurning = 0.5;

    // These two used for simulation currently
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.005;

  }

  // Swerve drive
  public static final class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = 0.635;

    // Distance between front and back wheels
    public static final double kWheelBase = kTrackWidth;

    // Need to update to correct values, I dont remember the value we set last meet
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // BL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); // BR

    // Driving Motor Ports
    public static final int kFrontLeftDriveMotorPort = 1; // Front Left
    public static final int kFrontRightDriveMotorPort = 2; // Front Right
    public static final int kBackLeftDriveMotorPort = 3; // Back Left
    public static final int kBackRightDriveMotorPort = 4; // Back Right

    // Turning Motor Ports
    public static final int kFrontLeftTurningMotorPort = 5; // Front Left
    public static final int kFrontRightTurningMotorPort = 6;// Front Right
    public static final int kBackLeftTurningMotorPort = 7; // Back Left
    public static final int kBackRightTurningMotorPort = 8; // Back Right

    // -------> ABE <-------- //
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
    public static final int kBackRightDriveAbsoluteEncoderPort = 4;
    // -------> ABE <-------- //

    // Need to update values for our specific magnetic fields
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = -0.411865;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = -0.185059;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -0.801758;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = -0.241943;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final double kPThetaController = 0.001;
    public static final double kIThetaController = 0.0;
    public static final double kDThetaController = 0.00;

    public static final double kMaxDriveMotorTemp = 33.0;

    public static final double kMotorMaxOutput = 1;

  }

  public static final class AutoConstants {
    public static final double kAutoDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), // Translation constants
        new PIDConstants(3, 0, 0), // Rotation constants
        kAutoDriveMaxSpeedMetersPerSecond,
        // Drive base radius (distance from center to furthest module)
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm(),
        new ReplanningConfig());
  }
}
