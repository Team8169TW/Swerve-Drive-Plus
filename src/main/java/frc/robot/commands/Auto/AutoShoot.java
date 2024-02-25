// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RunMode;
import frc.robot.Constants.ShooterConstants.SpeedSet;
import frc.robot.commands.Intake.IntakeNormal;
import frc.robot.commands.Shooter.ShooterNormal;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelDeadlineGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(
      ShooterSubsystem shooterSubsystem,
      LinkageSubsystem linkageSubsystem,
      IntakeSubsystem intakeSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(2));
    addCommands(
        new ShooterNormal(shooterSubsystem, linkageSubsystem, null, SpeedSet.kSpeak),
        new SequentialCommandGroup(
            new WaitCommand(1),
            new IntakeNormal(intakeSubsystem, RunMode.kFwd)));
  }
}
