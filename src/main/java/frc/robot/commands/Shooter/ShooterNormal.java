// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterNormal extends Command {
  ShooterSubsystem shooterSubsystem;
  BooleanSupplier onStop;

  /** Creates a new ShooterNormal. */
  public ShooterNormal(ShooterSubsystem shooterSubsystem, BooleanSupplier onStop) {
    this.shooterSubsystem = shooterSubsystem;
    this.onStop = onStop;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter RPM", ShooterConstants.kShooterMotorDefaultRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int rpm = (int)SmartDashboard.getNumber("Shooter RPM", ShooterConstants.kShooterMotorDefaultRPM);
    System.out.println(rpm);
    shooterSubsystem.setSpeed(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onStop.getAsBoolean();
  }
}
