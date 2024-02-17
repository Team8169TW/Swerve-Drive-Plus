// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.RunMode;
import frc.robot.Constants.ShooterConstants.SpeedSet;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeFromHead;
import frc.robot.commands.Intake.IntakeNormal;
import frc.robot.commands.Linkage.LinkageAuto;
import frc.robot.commands.Linkage.LinkageNormal;
import frc.robot.commands.Shooter.ShooterNormal;
import frc.robot.commands.Swerve.SwerveNormal;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinkageSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static CommandXboxController driverController = new CommandXboxController(IOConstants.kControllerDriver);
  private static CommandXboxController operatorController = new CommandXboxController(IOConstants.kControllerOperator);
  private static XboxController operatorControllerNC = new XboxController(IOConstants.kControllerOperator);

  // Create subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LinkageSubsystem linkageSubsystem = new LinkageSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Create auto chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommand();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake fine
    operatorController.b().whileTrue(new IntakeNormal(intakeSubsystem, RunMode.kFwd));
    operatorController.a().whileTrue(new IntakeNormal(intakeSubsystem, RunMode.kRev));
    // Intake auto
    operatorController.y().toggleOnTrue(new IntakeAuto(intakeSubsystem, linkageSubsystem));
    // Intake from head
    operatorController.x().toggleOnTrue(new IntakeFromHead(intakeSubsystem, shooterSubsystem, linkageSubsystem));

    // Linkage fine
    driverController.leftBumper().whileTrue(new LinkageNormal(linkageSubsystem, RunMode.kUp));
    driverController.rightBumper().whileTrue(new LinkageNormal(linkageSubsystem, RunMode.kDown));
    // Linkage auto
    driverController.pov(0).onTrue(new LinkageAuto(linkageSubsystem, RunMode.kShoot));
    driverController.pov(90).onTrue(new LinkageAuto(linkageSubsystem, RunMode.kIdle));
    driverController.pov(180).onTrue(new LinkageAuto(linkageSubsystem, RunMode.kIntake));

    // Shooter
    // operatorController.start().onTrue(new ShooterNormal(shooterSubsystem, operatorControllerNC::getBackButton, linkageSubsystem));
    operatorController.pov(0).onTrue(new ShooterNormal(shooterSubsystem, linkageSubsystem, operatorControllerNC::getBackButton, SpeedSet.kSpeak));
    operatorController.pov(90).onTrue(new ShooterNormal(shooterSubsystem, linkageSubsystem, operatorControllerNC::getBackButton, SpeedSet.kAmp));
    operatorController.pov(180).onTrue(new ShooterNormal(shooterSubsystem, linkageSubsystem, operatorControllerNC::getBackButton, SpeedSet.kStage));
    operatorController.pov(270).onTrue(new ShooterNormal(shooterSubsystem, linkageSubsystem, operatorControllerNC::getBackButton, SpeedSet.kManual));

    // Disable
    // operatorController.leftBumper().onTrue(new InstantCommand(()->{System.out.println(0/0);}));
  }

  private void setDefaultCommand() {
    swerveSubsystem.setDefaultCommand(new SwerveNormal(swerveSubsystem,
        () -> -driverController.getLeftY(), // X-Axis
        () -> -driverController.getLeftX(), // Y-Axis
        () -> -driverController.getRightX() // R-Axis
    ));

    // linkageSubsystem.setDefaultCommand(new LinkageAuto(linkageSubsystem, RunMode.kIdle));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("start shoot speak", new ShooterNormal(shooterSubsystem, linkageSubsystem, operatorControllerNC::getBackButton, SpeedSet.kSpeak));
    NamedCommands.registerCommand("intake", new IntakeNormal(intakeSubsystem, RunMode.kFwd));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
