// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Linkage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RunMode;
import frc.robot.subsystems.LinkageSubsystem;

public class LinkageAuto extends Command {
  LinkageSubsystem linkageSubsystem;
  RunMode mode;

  /** Creates a new LinkageAuto. */
  public LinkageAuto(LinkageSubsystem linkageSubsystem, RunMode mode) {
    this.linkageSubsystem = linkageSubsystem;
    this.mode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linkageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mode == RunMode.kIdle)
      linkageSubsystem.setIdle();
    if (mode == RunMode.kShoot)
      linkageSubsystem.setShooter();
    if (mode == RunMode.kIntake)
      linkageSubsystem.setIntaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
