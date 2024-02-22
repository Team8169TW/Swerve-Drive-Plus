// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.Constants.LinkageConstants.LinkageState;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class StatusSubsystem extends SubsystemBase {
  private static boolean swerveAutoByIntake = false;
  private static boolean swerveAutoByIntakeSig = false;
  private static boolean swerveAutoByShooter = false;
  private static boolean swerveAutoByShooterSig = false;
  private static IntakeState intakeState = IntakeState.kStop;
  private static boolean intakeStateSig = false;
  private static ShooterState shooterStateT = ShooterState.kStop;
  private static boolean shooterStateSigT = false;
  private static ShooterState shooterStateB = ShooterState.kStop;
  private static boolean shooterStateSigB = false;
  private static LinkageState linkageState = LinkageState.kOk;
  private static boolean linkageStateSig = false;

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

    if (intakeState == IntakeState.kStop) {
      intakeStateSig = false;
    } else if (intakeState == IntakeState.kRunning) {
      intakeStateSig = true;
    } else {
      intakeStateSig = !intakeStateSig;
    }
    SmartDashboard.putBoolean("Intake state", intakeStateSig);

    if (shooterStateT == ShooterState.kStop) {
      shooterStateSigT = false;
    } else if (shooterStateT == ShooterState.kReady) {
      shooterStateSigT = true;
    } else {
      shooterStateSigT = !shooterStateSigT;
    }
    SmartDashboard.putBoolean("Shooter state T", shooterStateSigT);
    if (shooterStateB == ShooterState.kStop) {
      shooterStateSigB = false;
    } else if (shooterStateB == ShooterState.kReady) {
      shooterStateSigB = true;
    } else {
      shooterStateSigB = !shooterStateSigB;
    }
    SmartDashboard.putBoolean("Shooter state B", shooterStateSigB);

    if (linkageState == LinkageState.kOk) {
      linkageStateSig = true;
    } else {
      linkageStateSig = !linkageStateSig;
    }
    SmartDashboard.putBoolean("Linkage state", linkageStateSig);
  }

  public static void setSwerveAuto(boolean on, Limelight by) {
    if (by == Limelight.kInatke) {
      swerveAutoByIntake = on;
    } else {
      swerveAutoByShooter = on;
    }
  }

  public static void setIntake(IntakeState state) {
    intakeState = state;
  }

  public static void setSooterT(ShooterState state) {
    shooterStateT = state;
  }

  public static void setSooterB(ShooterState state) {
    shooterStateB = state;
  }

  public static void setLinkage(LinkageState state) {
    linkageState = state;
  }
}
