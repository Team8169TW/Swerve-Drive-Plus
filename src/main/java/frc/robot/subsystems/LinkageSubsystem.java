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
import frc.robot.Constants.RunMode;

public class LinkageSubsystem extends SubsystemBase {
  private final CANSparkMax linkageMotor = new CANSparkMax(LinkageConstants.kLinkageMotorPort, MotorType.kBrushless);
  private final SparkAbsoluteEncoder linkageEncoder = linkageMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController linkagePIDController = linkageMotor.getPIDController();

  private static RunMode state;

  private final double deadband = 0.001;

  /** Creates a new LinkageSubsystem. */
  public LinkageSubsystem() {
    linkageMotor.setInverted(false);
    linkageMotor.setIdleMode(IdleMode.kBrake);

    // Set PID values for the Spark Max PID
    linkagePIDController.setPositionPIDWrappingEnabled(false);
    linkagePIDController.setP(LinkageConstants.kPLinkage);
    linkagePIDController.setI(LinkageConstants.kILinkage);
    linkagePIDController.setD(LinkageConstants.kDLinkage);
    linkagePIDController.setIZone(0.0);
    linkagePIDController.setFF(0.0);
    linkagePIDController.setOutputRange(-0.5, 0.35);
    linkagePIDController.setFeedbackDevice(linkageEncoder);
    linkageMotor.burnFlash();
  }

  public double getPosition() {
    return linkageEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Linkage Position", getPosition());

    if (state == RunMode.kUp && Math.abs(getPosition() - LinkageConstants.kUpPosition) < deadband) {
      stop();
    }

    if (state == RunMode.kDown && Math.abs(getPosition() - LinkageConstants.kDownPosition) < deadband) {
      stop();
    }
  }

  public void up() {
    state = RunMode.kUp;
    linkagePIDController.setReference(LinkageConstants.kUpPosition, ControlType.kPosition);
  }

  public void down() {
    state = RunMode.kDown;
    linkagePIDController.setReference(LinkageConstants.kDownPosition, ControlType.kPosition);
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
