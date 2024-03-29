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

    public static final double kDeadband = 0.05;

    public static final int kControllerDriver = 1;
    public static final int kControllerOperator = 0;

    public static double deadbandHandler(double value, double deadband) {
      if (Math.abs(value) < deadband) {
        return 0;
      } else if (value > 0) {
        return (value - IOConstants.kDeadband) / (1 - IOConstants.kDeadband);
      } else {
        return (value + IOConstants.kDeadband) / (1 - IOConstants.kDeadband);
      }
    }
  }

  // Swerve modules
  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1.0 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);
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
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR

    // Driving Motor Ports
    public static final int kFrontLeftDriveMotorPort = 4; // Front Left
    public static final int kFrontRightDriveMotorPort = 3; // Front Right
    public static final int kBackLeftDriveMotorPort = 2; // Back Left
    public static final int kBackRightDriveMotorPort = 1; // Back Right

    // Turning Motor Ports
    public static final int kFrontLeftTurningMotorPort = 8; // Front Left
    public static final int kFrontRightTurningMotorPort = 7;// Front Right
    public static final int kBackLeftTurningMotorPort = 6; // Back Left
    public static final int kBackRightTurningMotorPort = 5; // Back Right

    // -------> ABE <-------- //
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;
    // -------> ABE <-------- //

    // Need to update values for our specific magnetic fields
    // public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg =
    // -0.411865;
    // public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg =
    // -0.185059;
    // public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg =
    // -0.801758;
    // public static final double kBackRightDriveAbsoluteEncoderOffsetDeg =
    // -0.241943;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final double kPTheta = 0.012;
    public static final double kITheta = 0.01;
    public static final double kDTheta = 0.00015;
    public static final double kIZTheta = 60.0;

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

  // Intake
  public static final class IntakeConstants {

    // Intake Motor Ports
    public static final int kIntakeMotorPort = 10;

    // Intake Motor Rate
    public static final double kIntakeMotorRateFwd = 0.5;
    public static final double kIntakeMotorRateRev = 0.5;

    // Intake Sensor Gate Value
    public static final int kIntakeGateValue = 300;

    public enum IntakeState {
      kRunning,
      kBlocked,
      kStop,
    }
  }

  // Shooter
  public static final class ShooterConstants {

    // Shooter Motor Ports
    public static final int kTopShooterMotorPort = 12;
    public static final int kBottomShooterMotorPort = 11;

    // Shooter Motor Ratio
    public static final double kShooterMotorGearRatio = 8.0 / 7.0;

    // Shooter Motor Speed
    public static final int kShooterMaxRPM = (int) Math.floor(5700 * kShooterMotorGearRatio);
    public static final int kShooterMotorDefaultRPM = 3600;

    // Shooter Motor PID
    public static final double kP = 0.0007;
    public static final double kI = 0.0000005;
    public static final double kD = 0.00001;
    public static final double kIZone = 200;
    public static final double kFF = 0.00018;
    public static final double kRamprate = 50000;

    // Intake Motor Rate
    public static final double kShooterMotorRateRev = 0.15;

    public enum SpeedSet {
      kSpeak(3600, 4600),
      kAmp(800, 850),
      kStage(875, 3795),
      kManual(5000, 3200),
      kSpeak1m(5000, 3300),
      kSpeak2m(5000, 2200);

      public final int topSpeed;
      public final int bottomSpeed;

      private SpeedSet(int topSpeed, int bottomSpeed) {
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
      }
    }

    public enum ShooterState {
      kReady,
      kPreparing,
      kStop,
    }
  }

  // Linkage
  public static final class LinkageConstants {

    // Linkage Motor Ports
    public static final int kLinkageLMotorPort = 13;
    public static final int kLinkageRMotorPort = 9;

    // Linkage Motor PID
    public static final double kP = 0.02; // 0.02
    public static final double kI = 0.00001; // 0.00005
    public static final double kD = 0.00001; // 0.005
    public static final double kIz = 0; // 20
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.8;
    public static final double kMinOutput = -1;

    // Linkage Motor Rate
    public static final double kLinkageMotorRateFine = 0.4;

    // Linkage Motor Ratio
    public static final double kLinkageMotorGearRatio = 100.0 / 14 * 46;

    // Linkage Motor Position
    public static final double kShootPosition = 234.00;
    public static final double kIdlePosition = 264.00;
    public static final double kIntakePosition = 288.50;

    // Linkage Motor Limit
    public static final double kUpLimit = kShootPosition - 1;
    public static final double kDownLimit = kIntakePosition + 1;

    public enum LinkageState {
      kOk,
      kAdj,
    }
  }

  public enum RunMode {
    kStop,
    kUp,
    kDown,
    kFwd,
    kRev,
    kIdle,
    kShoot,
    kIntake,
  }

  // Limelight
  public static final class LimelightConstants {

    // Limelight Name Mapping
    public enum Limelight {
      kShooter("limelight-b", 1), // IP: 10.81.69.13
      kInatke("limelight-c", -1); // IP: 10.81.69.15

      public final String hostname;
      public final int approachingXSpeed;

      private Limelight(String hostname, int approachingXSpeed) {
        this.hostname = hostname;
        this.approachingXSpeed = approachingXSpeed;
      }
    }
  }
}
