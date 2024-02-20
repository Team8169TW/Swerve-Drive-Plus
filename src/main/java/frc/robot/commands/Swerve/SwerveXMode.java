// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveXMode extends Command {
  SwerveSubsystem swerveSubsystem;

  /** Creates a new SwerveXMode. */
  public SwerveXMode(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(45));
    desiredStates[1] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(135));
    desiredStates[2] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(135));
    desiredStates[3] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(45));
    swerveSubsystem.setModuleStates(desiredStates);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
