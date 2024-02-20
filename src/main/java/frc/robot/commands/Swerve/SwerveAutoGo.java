// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAutoGo extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier speed;
  private boolean detected;

  /** Creates a new SwerveAutoGo. */
  public SwerveAutoGo(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = null;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  public SwerveAutoGo(SwerveSubsystem swerveSubsystem, DoubleSupplier speed) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    detected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(LimelightConstants.kInatkeLL)) {
      detected = true;
      double turningAngle = -LimelightHelpers.getTX(LimelightConstants.kInatkeLL);
      double xSpeed;
      if (speed != null) {
        xSpeed = IOConstants.deadbandHandler(speed.getAsDouble(), 0.1) * 0.5;
      } else {
        xSpeed = LimelightHelpers.getTY(LimelightConstants.kInatkeLL) * 0.01;
      }
      swerveSubsystem.setChassisOutput(-xSpeed, 0, turningAngle, true, true);
    } else {
      swerveSubsystem.stopModules();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speed == null && detected && !LimelightHelpers.getTV(LimelightConstants.kInatkeLL);
  }
}
