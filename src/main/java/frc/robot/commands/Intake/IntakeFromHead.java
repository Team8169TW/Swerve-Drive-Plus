// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeFromHead extends Command {
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  LinkageSubsystem linkageSubsystem;
  Timer timer = new Timer();

  /** Creates a new IntakeFromHead. */
  public IntakeFromHead(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LinkageSubsystem linkageSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // this.linkageSubsystem = linkageSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
    // addRequirements(linkageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // linkageSubsystem.setShooter();
    shooterSubsystem.runRev();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(timer.get());
    if(intakeSubsystem.isPass() && timer.get()==0){
      timer.start();
      intakeSubsystem.runRev();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    shooterSubsystem.stop();
    intakeSubsystem.stop();
    // linkageSubsystem.setIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.10;
  }
}
