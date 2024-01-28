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

  private final RelativeEncoder shooterEncoderT = topShooterMotor.getEncoder();
  private final RelativeEncoder shooterEncoderB = bottomShooterMotor.getEncoder();

  private final SparkPIDController shooterPIDControllerT = topShooterMotor.getPIDController();
  private final SparkPIDController shooterPIDControllerB = topShooterMotor.getPIDController();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // bottomShooterMotor.follow(topShooterMotor);

    topShooterMotor.setInverted(false);
    bottomShooterMotor.setInverted(false);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    shooterEncoderT.setVelocityConversionFactor(ShooterConstants.kShooterMotorGearRatio);
    shooterEncoderB.setVelocityConversionFactor(ShooterConstants.kShooterMotorGearRatio);

    // Set PID values for the Spark Max PID
    shooterPIDControllerT.setP(ShooterConstants.kPShooter);
    shooterPIDControllerT.setI(ShooterConstants.kIShooter);
    shooterPIDControllerT.setD(ShooterConstants.kDShooter);
    shooterPIDControllerT.setIZone(0.0);
    shooterPIDControllerT.setFF(0.0);
    shooterPIDControllerT.setOutputRange(0, 1);
    topShooterMotor.burnFlash();

    // Set PID values for the Spark Max PID
    shooterPIDControllerB.setP(ShooterConstants.kPShooter);
    shooterPIDControllerB.setI(ShooterConstants.kIShooter);
    shooterPIDControllerB.setD(ShooterConstants.kDShooter);
    shooterPIDControllerB.setIZone(0.0);
    shooterPIDControllerB.setFF(0.0);
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
  }

  public void setSpeed(int speed) {
    if (speed > ShooterConstants.kShooterMotorMaxRPM)
      speed = ShooterConstants.kShooterMotorMaxRPM;
    // shooterPIDController.setReference(speed, ControlType.kVelocity);
    // topShooterMotor.set((double)3600/ShooterConstants.kShooterMotorMaxRPM);

    shooterPIDControllerT.setReference(ShooterConstants.kShooterMotorDefaultRPM, ControlType.kVelocity);
    shooterPIDControllerB.setReference(ShooterConstants.kShooterMotorDefaultRPM, ControlType.kVelocity);
  }

  public void stop() {
    topShooterMotor.set(0);
    bottomShooterMotor.set(0);
  }
}
