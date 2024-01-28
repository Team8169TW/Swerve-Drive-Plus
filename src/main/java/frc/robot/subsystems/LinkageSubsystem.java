// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinkageConstants;

public class LinkageSubsystem extends SubsystemBase {
  private final CANSparkMax linkageMotor = new CANSparkMax(LinkageConstants.kLinkageMotorPort,MotorType.kBrushless);
  private final SparkAbsoluteEncoder linkageEncoder = linkageMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkPIDController linkagePIDController = linkageMotor.getPIDController();

  /** Creates a new LinkageSubsystem. */
  public LinkageSubsystem() {
    linkageMotor.setInverted(false);
    linkageMotor.setIdleMode(IdleMode.kBrake);

    // Set PID values for the Spark Max PID
    linkagePIDController.setP(LinkageConstants.kPLinkage);
    linkagePIDController.setI(LinkageConstants.kILinkage);
    linkagePIDController.setD(LinkageConstants.kDLinkage);
    linkagePIDController.setIZone(0.0);
    linkagePIDController.setFF(0.0);
    linkagePIDController.setOutputRange(-1, 1);
    linkageMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Linkage Position", linkageEncoder.getVelocity());
  }

  public void up(){
    linkagePIDController.setReference(LinkageConstants.kUpPosition, ControlType.kPosition);
  }

  public void down(){
    linkagePIDController.setReference(LinkageConstants.kDownPosition, ControlType.kPosition);
  }

  public void stop(){
    linkageMotor.set(0);
  }
}
