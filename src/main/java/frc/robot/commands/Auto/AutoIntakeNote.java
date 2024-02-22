// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Swerve.SwerveAutoGo;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeNote extends ParallelDeadlineGroup {
  /** Creates a new AutoIntakeNote. */
  public AutoIntakeNote(
    SwerveSubsystem swerveSubsystem, 
    IntakeSubsystem intakeSubsystem,
    LinkageSubsystem linkageSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeAuto(intakeSubsystem, linkageSubsystem));
    addCommands(new SwerveAutoGo(swerveSubsystem, Limelight.kInatke));
  }
}
