// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SpeedSet;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterNormal extends Command {
  ShooterSubsystem shooterSubsystem;
  BooleanSupplier onStop;
  SpeedSet speed;
  LinkageSubsystem linkageSubsystem;

  /** Creates a new ShooterNormal. */
  public ShooterNormal(ShooterSubsystem shooterSubsystem, LinkageSubsystem linkageSubsystem, BooleanSupplier onStop, SpeedSet speed) {
    this.shooterSubsystem = shooterSubsystem;
    this.onStop = onStop;
    this.speed = speed;
    this.linkageSubsystem = linkageSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    addRequirements(linkageSubsystem);

    SmartDashboard.putNumber("Shooter Set RPM T",
        SmartDashboard.getNumber("Shooter Set RPM T", ShooterConstants.kShooterMotorDefaultRPM));
    SmartDashboard.putNumber("Shooter Set RPM B",
        SmartDashboard.getNumber("Shooter Set RPM B", ShooterConstants.kShooterMotorDefaultRPM));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linkageSubsystem.setShooter();
    shooterSubsystem.setSpeed(speed.topSpeed, speed.bottomSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed == SpeedSet.kManual) {
      int rpmT = (int) SmartDashboard.getNumber("Shooter Set RPM T", ShooterConstants.kShooterMotorDefaultRPM);
      int rpmB = (int) SmartDashboard.getNumber("Shooter Set RPM B", ShooterConstants.kShooterMotorDefaultRPM);
      shooterSubsystem.setSpeed(rpmT, rpmB);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    linkageSubsystem.setIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onStop.getAsBoolean();
  }
}
