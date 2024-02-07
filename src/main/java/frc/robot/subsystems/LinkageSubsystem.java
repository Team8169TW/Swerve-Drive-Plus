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
import frc.robot.Constants.RunMode;

public class LinkageSubsystem extends SubsystemBase {
  private final CANSparkMax linkageMotor = new CANSparkMax(LinkageConstants.kLinkageMotorPort, MotorType.kBrushless);
  private final RelativeEncoder linkageEncoder = linkageMotor.getEncoder();
  private final SparkAbsoluteEncoder linkageAbsEncoder = linkageMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController linkagePIDController = linkageMotor.getPIDController();

  private static RunMode state;

  private final double deadband = 0.01;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new LinkageSubsystem. */
  public LinkageSubsystem() {
    linkageMotor.setInverted(false);
    linkageMotor.setIdleMode(IdleMode.kBrake);

    linkageEncoder.setPosition(linkageAbsEncoder.getPosition());

    linkageMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    linkageMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    linkageMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) LinkageConstants.kDownLimit+1);
    linkageMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) LinkageConstants.kUpLimit-1);

    kP = LinkageConstants.kP;
    kI = LinkageConstants.kI;
    kD = LinkageConstants.kI;
    kIz = LinkageConstants.kIz;
    kFF = LinkageConstants.kFF;  
    kMaxOutput = LinkageConstants.kMaxOutput;
    kMinOutput = LinkageConstants.kMinOutput;

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

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
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

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

  public void up() {
    state = RunMode.kUp;
    linkagePIDController.setReference(LinkageConstants.kUpLimit, ControlType.kPosition);
  }

  public void down() {
    state = RunMode.kDown;
    linkagePIDController.setReference(LinkageConstants.kDownLimit, ControlType.kPosition);
  }

  public void upFine() {
    state = RunMode.kUp;
    linkageMotor.set(-LinkageConstants.kLinkageMotorRateFine);
  }

  public void downFine() {
    state = RunMode.kDown;
    linkageMotor.set(LinkageConstants.kLinkageMotorRateFine);
  }

  public void stop() {
    state = RunMode.kStop;
    linkageMotor.set(0);
  }

  public boolean isDone() {
    return state == RunMode.kStop;
  }
}