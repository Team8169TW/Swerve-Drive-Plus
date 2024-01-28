// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RunMode;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNormal extends Command {
private IntakeSubsystem intakeSubsystem;
private RunMode mode;

  /** Creates a new IntakeNormal. */
  public IntakeNormal(IntakeSubsystem intakeSubsystem, RunMode mode) {
    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mode == RunMode.kFwd)
      intakeSubsystem.runFwd();
    if(mode == RunMode.kRev)
      intakeSubsystem.runRev();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
