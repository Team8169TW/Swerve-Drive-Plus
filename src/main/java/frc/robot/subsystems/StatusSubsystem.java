// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants.Limelight;

public class StatusSubsystem extends SubsystemBase {
  private static boolean swerveAutoByIntake = false;
  private static boolean swerveAutoByIntakeSig = false;
  private static boolean swerveAutoByShooter = false;
  private static boolean swerveAutoByShooterSig = false;

  /** Creates a new StatusSubsystem. */
  public StatusSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (swerveAutoByIntake) {
      swerveAutoByIntakeSig = !swerveAutoByIntakeSig;
    } else {
      swerveAutoByIntakeSig = false;
    }
    SmartDashboard.putBoolean("Swerve AutoByIntake", swerveAutoByIntakeSig);

    if (swerveAutoByShooter) {
      swerveAutoByShooterSig = !swerveAutoByShooterSig;
    } else {
      swerveAutoByShooterSig = false;
    }
    SmartDashboard.putBoolean("Swerve AutoByShooter", swerveAutoByShooterSig);
  }

  public static void setSwerveAuto(boolean on, Limelight by) {
    if (by == Limelight.kInatke) {
      swerveAutoByIntake = on;
    } else {
      swerveAutoByShooter = on;
    }
  }
}
