// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinkageConstants;

public class LinkageSubsystem extends SubsystemBase {
  private final CANSparkMax linkageMotor = new CANSparkMax(LinkageConstants.kLinkageMotorPort, MotorType.kBrushless);
  private final RelativeEncoder linkageEncoder = linkageMotor.getEncoder();
  private final SparkAbsoluteEncoder linkageAbsEncoder = linkageMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController linkagePIDController = linkageMotor.getPIDController();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new LinkageSubsystem. */
  public LinkageSubsystem() {
    linkageMotor.setInverted(true);
    linkageMotor.setIdleMode(IdleMode.kBrake);

    linkageEncoder.setPosition(linkageAbsEncoder.getPosition());

    linkageMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    linkageMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    linkageMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) LinkageConstants.kDownLimit);
    linkageMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) LinkageConstants.kUpLimit);

    kP = LinkageConstants.kP;
    kI = LinkageConstants.kI;
    kD = LinkageConstants.kI;
    kIz = LinkageConstants.kIz;
    kFF = LinkageConstants.kFF;  
    kMaxOutput = LinkageConstants.kMaxOutput;
    kMinOutput = LinkageConstants.kMinOutput;

    SmartDashboard.putNumber("Linkage P Gain", kP);
    SmartDashboard.putNumber("Linkage I Gain", kI);
    SmartDashboard.putNumber("Linkage D Gain", kD);
    SmartDashboard.putNumber("Linkage I Zone", kIz);
    SmartDashboard.putNumber("Linkage Feed Forward", kFF);
    SmartDashboard.putNumber("Linkage Max Output", kMaxOutput);
    SmartDashboard.putNumber("Linkage Min Output", kMinOutput);

    // Set PID values for the Spark Max PID
    linkagePIDController.setP(kP);
    linkagePIDController.setI(kI);
    linkagePIDController.setD(kD);
    linkagePIDController.setIZone(kIz);
    linkagePIDController.setFF(kFF);
    linkagePIDController.setOutputRange(kMinOutput, kMaxOutput);
    linkagePIDController.setFeedbackDevice(linkageAbsEncoder);
    linkageMotor.burnFlash();
  }

  public double getAbsPosition() {
    return linkageAbsEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Linkage Abs Position", getAbsPosition());

    // if (state == RunMode.kUp && Math.abs(getAbsPosition() - LinkageConstants.kUpLimit) < deadband) {
    //   stop();
    // }

    // if (state == RunMode.kDown && Math.abs(getAbsPosition() - LinkageConstants.kDownLimit) < deadband) {
    //   stop();
    // }

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Linkage P Gain", 0);
    double i = SmartDashboard.getNumber("Linkage I Gain", 0);
    double d = SmartDashboard.getNumber("Linkage D Gain", 0);
    double iz = SmartDashboard.getNumber("Linkage I Zone", 0);
    double ff = SmartDashboard.getNumber("Linkage Feed Forward", 0);
    double max = SmartDashboard.getNumber("Linkage Max Output", 0);
    double min = SmartDashboard.getNumber("Linkage Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { linkagePIDController.setP(p); kP = p; }
    if((i != kI)) { linkagePIDController.setI(i); kI = i; }
    if((d != kD)) { linkagePIDController.setD(d); kD = d; }
    if((iz != kIz)) { linkagePIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { linkagePIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      linkagePIDController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }
  }

  public void setIdle(){
    linkagePIDController.setReference(LinkageConstants.kIdlePosition, ControlType.kPosition);
  }

  public void setShooter(){
    linkagePIDController.setReference(LinkageConstants.kShootPosition, ControlType.kPosition);
  }

  public void setIntaker(){
    linkagePIDController.setReference(LinkageConstants.kIntakePosition, ControlType.kPosition);
  }

  public void upFine() {
    linkageMotor.set(-LinkageConstants.kLinkageMotorRateFine);
  }

  public void downFine() {
    linkageMotor.set(LinkageConstants.kLinkageMotorRateFine);
  }

  public void stop() {
    linkageMotor.set(0);
  }

  public void hold(){
    linkagePIDController.setReference(getAbsPosition(), ControlType.kPosition);
  }
}
