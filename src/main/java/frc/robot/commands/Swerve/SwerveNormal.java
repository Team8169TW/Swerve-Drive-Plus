// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.Swerve;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveNormal extends Command {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  // Command constructor
  public SwerveNormal(
      SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction) {

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Running loop of command
  @Override
  public void execute() {

    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningAngle = turningSpdFunction.get();

    // Apply deadband to protect motors
    xSpeed = IOConstants.deadbandHandler(xSpeed, IOConstants.kDeadband);
    ySpeed = IOConstants.deadbandHandler(ySpeed, IOConstants.kDeadband);
    turningAngle = Math.abs(turningAngle) > IOConstants.kDeadband ? turningAngle / (1 - IOConstants.kDeadband) : 0.0;

    // Apply slew rate to joystick input to make robot input smoother and mulitply
    // by max speed
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    turningAngle = turningLimiter.calculate(turningAngle);

    turningAngle *= 5;

    swerveSubsystem.setChassisOutput(xSpeed, ySpeed, turningAngle);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
