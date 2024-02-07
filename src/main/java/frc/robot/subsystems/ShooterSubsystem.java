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

  private SlewRateLimiter normalRateLimiter = new SlewRateLimiter(ShooterConstants.kRamprate);
  private SlewRateLimiter stopRateLimiter = new SlewRateLimiter(ShooterConstants.kRamprate / 8);

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double setPoint;

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

    // Set PID values for the Spark Max PID
    shooterPIDControllerT.setP(ShooterConstants.kP);
    shooterPIDControllerT.setI(ShooterConstants.kI);
    shooterPIDControllerT.setD(ShooterConstants.kD);
    shooterPIDControllerT.setIZone(ShooterConstants.kIZone);
    shooterPIDControllerT.setFF(ShooterConstants.kFF);
    shooterPIDControllerT.setOutputRange(0, 1);
    topShooterMotor.burnFlash();

    // Set PID values for the Spark Max PID
    shooterPIDControllerB.setP(ShooterConstants.kP);
    shooterPIDControllerB.setI(ShooterConstants.kI);
    shooterPIDControllerB.setD(ShooterConstants.kD);
    shooterPIDControllerB.setIZone(ShooterConstants.kIZone);
    shooterPIDControllerB.setFF(ShooterConstants.kFF);
    shooterPIDControllerB.setOutputRange(0, 1);
    bottomShooterMotor.burnFlash();

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM T", shooterEncoderT.getVelocity());
    SmartDashboard.putNumber("Shooter RPM B", shooterEncoderB.getVelocity());

    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    // if((p != kP)) { shooterPIDController.setP(p); kP = p; }
    // if((i != kI)) { shooterPIDController.setI(i); kI = i; }
    // if((d != kD)) { shooterPIDController.setD(d); kD = d; }
    // if((iz != kIz)) { shooterPIDController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { shooterPIDController.setFF(ff); kFF = ff; }

    if (setPoint >= 0) {
      // Calculate and set new reference RPM
      double reference_setpoint;
      if (setPoint == 0) {
        // when we hit stop, stop immediately. (safety!)
        reference_setpoint = stopRateLimiter.calculate(setPoint);
        normalRateLimiter.reset(reference_setpoint);
        // topShooterMotor.set(0);
        // bottomShooterMotor.set(0);
      } else {
        reference_setpoint = normalRateLimiter.calculate(setPoint);
        stopRateLimiter.reset(reference_setpoint);
      }
      shooterPIDControllerT.setReference(reference_setpoint, ControlType.kVelocity);
      shooterPIDControllerB.setReference(reference_setpoint, ControlType.kVelocity);
    }
  }

  public void setSpeed(int speed) {
    if (speed > ShooterConstants.kShooterMotorMaxRPM)
      speed = ShooterConstants.kShooterMotorMaxRPM;

    setPoint = speed;
    // shooterPIDController.setReference(speed, ControlType.kVelocity);
    // topShooterMotor.set((double)3600/ShooterConstants.kShooterMotorMaxRPM);

    // shooterPIDControllerT.setReference(ShooterConstants.kShooterMotorDefaultRPM,
    // ControlType.kVelocity);
    // shooterPIDControllerB.setReference(ShooterConstants.kShooterMotorDefaultRPM,
    // ControlType.kVelocity);
  }

  public void runRev() {
    topShooterMotor.set(-ShooterConstants.kShooterMotorRateRev);
    bottomShooterMotor.set(-ShooterConstants.kShooterMotorRateRev);
    setPoint = -1;
  }

  public void stop() {
    topShooterMotor.set(0);
    bottomShooterMotor.set(0);
  }
}