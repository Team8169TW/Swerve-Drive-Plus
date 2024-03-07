// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.StatusSubsystem;

public class IntakeAuto extends Command {
  private IntakeSubsystem intakeSubsystem;
  private LinkageSubsystem linkageSubsystem;
  Timer timer = new Timer();

  /** Creates a new IntakeAuto. */
  public IntakeAuto(IntakeSubsystem intakeSubsystem, LinkageSubsystem linkageSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // this.linkageSubsystem = linkageSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    // addRequirements(linkageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // linkageSubsystem.setIntaker();
    intakeSubsystem.runFwd();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeSubsystem.isPass() && timer.get()==0){
      timer.start();
      intakeSubsystem.runRev();
      StatusSubsystem.setNotePassed(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    intakeSubsystem.stop();
    // linkageSubsystem.setIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.10;
  }
}
