// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(ShooterConstants.kBottomShooterMotorPort,
      MotorType.kBrushless);

  private final RelativeEncoder shooterEncoderT = topShooterMotor.getEncoder();
  private final RelativeEncoder shooterEncoderB = bottomShooterMotor.getEncoder();

  private final SparkPIDController shooterPIDControllerT = topShooterMotor.getPIDController();
  private final SparkPIDController shooterPIDControllerB = bottomShooterMotor.getPIDController();

  private SlewRateLimiter topRateLimiter = new SlewRateLimiter(ShooterConstants.kRamprate);
  private SlewRateLimiter bottomRateLimiter = new SlewRateLimiter(ShooterConstants.kRamprate);

  public double kP, kI, kD, kIZone, kFF, maxRPM;
  public double setPointTop, setPointBottom;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.setInverted(false);
    bottomShooterMotor.setInverted(false);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    shooterEncoderT.setVelocityConversionFactor(ShooterConstants.kShooterMotorGearRatio);
    shooterEncoderB.setVelocityConversionFactor(ShooterConstants.kShooterMotorGearRatio);

    // bottomShooterMotor.follow(topShooterMotor);

    kP = ShooterConstants.kP;
    kI = ShooterConstants.kI;
    kD = ShooterConstants.kD;
    kIZone = ShooterConstants.kIZone;
    kFF = ShooterConstants.kFF;

    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter I Zone", kIZone);
    SmartDashboard.putNumber("Shooter Feed Forward", kFF);

    // Set PID values for the Spark Max PID
    shooterPIDControllerT.setP(kP);
    shooterPIDControllerT.setI(kI);
    shooterPIDControllerT.setD(kD);
    shooterPIDControllerT.setIZone(kIZone);
    shooterPIDControllerT.setFF(kFF);
    shooterPIDControllerT.setOutputRange(0, 1);
    topShooterMotor.burnFlash();

    // Set PID values for the Spark Max PID
    shooterPIDControllerB.setP(kP);
    shooterPIDControllerB.setI(kI);
    shooterPIDControllerB.setD(kD);
    shooterPIDControllerB.setIZone(kIZone);
    shooterPIDControllerB.setFF(kFF);
    shooterPIDControllerB.setOutputRange(0, 1);
    bottomShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM T", shooterEncoderT.getVelocity());
    SmartDashboard.putNumber("Shooter RPM B", shooterEncoderB.getVelocity());

    SmartDashboard.putBoolean("Shooter OK T", Math.abs(shooterEncoderT.getVelocity() - setPointTop) < 50);
    SmartDashboard.putBoolean("Shooter OK B", Math.abs(shooterEncoderB.getVelocity() - setPointBottom) < 50);

    // // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter D Gain", 0);
    double iz = SmartDashboard.getNumber("Shooter I Zone", 0);
    double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { shooterPIDControllerT.setP(p); shooterPIDControllerB.setP(p); kP = p; }
    if((i != kI)) { shooterPIDControllerT.setI(i); shooterPIDControllerB.setI(i); kI = i; }
    if((d != kD)) { shooterPIDControllerT.setD(d); shooterPIDControllerB.setD(d); kD = d; }
    if((iz != kIZone)) { shooterPIDControllerT.setIZone(iz); shooterPIDControllerB.setIZone(iz); kIZone = iz; }
    if((ff != kFF)) { shooterPIDControllerT.setFF(ff); shooterPIDControllerB.setFF(ff); kFF = ff; }

    if (setPointTop >= 0) {
      // Calculate and set new reference RPM
      double reference_setpoint;
      reference_setpoint = topRateLimiter.calculate(setPointTop);
      shooterPIDControllerT.setReference(reference_setpoint, ControlType.kVelocity);
    }

    if (setPointBottom >= 0) {
      // Calculate and set new reference RPM
      double reference_setpoint;
      reference_setpoint = bottomRateLimiter.calculate(setPointBottom);
      shooterPIDControllerB.setReference(reference_setpoint, ControlType.kVelocity);
    }
  }

  public void setSpeed(int topSpeed, int bottomSpeed) {
    if (topSpeed > ShooterConstants.kShooterMotorMaxRPM)
      topSpeed = ShooterConstants.kShooterMotorMaxRPM;
    setPointTop = topSpeed;
    if (bottomSpeed > ShooterConstants.kShooterMotorMaxRPM)
      bottomSpeed = ShooterConstants.kShooterMotorMaxRPM;
    setPointBottom = bottomSpeed;
  }

  public void runRev() {
    topShooterMotor.set(-ShooterConstants.kShooterMotorRateRev);
    bottomShooterMotor.set(-ShooterConstants.kShooterMotorRateRev);
    setPointTop = -1;
    setPointBottom = -1;
  }

  public void stop() {
    topShooterMotor.set(0);
    bottomShooterMotor.set(0);
    setPointTop = 0;
    setPointBottom = 0;
    topRateLimiter.reset(0);
    bottomRateLimiter.reset(0);
  }
}
