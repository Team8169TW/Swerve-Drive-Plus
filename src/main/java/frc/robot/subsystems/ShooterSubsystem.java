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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topShooterMotor = new CANSparkMax(ShooterConstants.kTopShooterMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(ShooterConstants.kBottomShooterMotorPort,
      MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = topShooterMotor.getEncoder();
  
  private SparkPIDController shooterPIDController = topShooterMotor.getPIDController();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    bottomShooterMotor.follow(topShooterMotor);

    topShooterMotor.setInverted(true);
    topShooterMotor.setIdleMode(IdleMode.kCoast);

    shooterEncoder.setVelocityConversionFactor(ShooterConstants.kShooterMotorGearRatio);

    // Set PID values for the Spark Max PID
    shooterPIDController.setP(ShooterConstants.kPShooter);
    shooterPIDController.setI(ShooterConstants.kIShooter);
    shooterPIDController.setD(ShooterConstants.kDShooter);
    shooterPIDController.setIZone(0.0);
    shooterPIDController.setFF(0.0);
    shooterPIDController.setOutputRange(0, 1);
    topShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
  }

  public void setSpeed(int speed) {
    if (speed > ShooterConstants.kShooterMotorMaxRPM)
      speed = ShooterConstants.kShooterMotorMaxRPM;
    shooterPIDController.setReference(speed, ControlType.kVelocity);
  }

  public void stop() {
    topShooterMotor.set(0);
  }
}
