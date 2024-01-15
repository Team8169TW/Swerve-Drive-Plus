// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private static CANSparkMax m_motorDA = new CANSparkMax(Constants.PORT_MOTOR_DA, MotorType.kBrushless);
  private static CANSparkMax m_motorDB = new CANSparkMax(Constants.PORT_MOTOR_DB, MotorType.kBrushless);
  private static CANSparkMax m_motorDC = new CANSparkMax(Constants.PORT_MOTOR_DC, MotorType.kBrushless);
  private static CANSparkMax m_motorDD = new CANSparkMax(Constants.PORT_MOTOR_DD, MotorType.kBrushless);
  private static CANSparkMax m_motorRA = new CANSparkMax(Constants.PORT_MOTOR_RA, MotorType.kBrushless);
  private static CANSparkMax m_motorRB = new CANSparkMax(Constants.PORT_MOTOR_RB, MotorType.kBrushless);
  private static CANSparkMax m_motorRC = new CANSparkMax(Constants.PORT_MOTOR_RC, MotorType.kBrushless);
  private static CANSparkMax m_motorRD = new CANSparkMax(Constants.PORT_MOTOR_RD, MotorType.kBrushless);
  private static RelativeEncoder m_encoderDA = m_motorDA.getEncoder();
  private static RelativeEncoder m_encoderDB = m_motorDB.getEncoder();
  private static RelativeEncoder m_encoderDC = m_motorDC.getEncoder();
  private static RelativeEncoder m_encoderDD = m_motorDD.getEncoder();
  //private static RelativeEncoder m_encoderRA = m_motorRA.getEncoder();
  //private static RelativeEncoder m_encoderRB = m_motorRB.getEncoder();
  //private static RelativeEncoder m_encoderRC = m_motorRC.getEncoder();
  //private static RelativeEncoder m_encoderRD = m_motorRD.getEncoder();
  private static CANCoder m_encoderRA = new CANCoder(Constants.PORT_ENCODER_1);
  private static CANCoder m_encoderRB = new CANCoder(Constants.PORT_ENCODER_2);
  private static CANCoder m_encoderRC = new CANCoder(Constants.PORT_ENCODER_3);
  private static CANCoder m_encoderRD = new CANCoder(Constants.PORT_ENCODER_4);
  private static AHRS m_ahrs = new AHRS(SPI.Port.kMXP);

  private static double N_D_SPEED = 0.2;
  
  public DriveTrain() {
    //setPositionZero();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AP", getModuleRAPosition());
    SmartDashboard.putNumber("AA", getModuleRAAngle());
  }

  @Override
  public void simulationPeriodic() {
  }

  public static void setModuleASpeed(double speedD, double speedR) {
    m_motorDA.set(speedD*N_D_SPEED);
    m_motorRA.set(speedR);
  }

  public static void setModuleBSpeed(double speedD, double speedR) {
    m_motorDB.set(speedD*N_D_SPEED);
    m_motorRB.set(speedR);
  }

  public static void setModuleCSpeed(double speedD, double speedR) {
    m_motorDC.set(speedD*N_D_SPEED);
    m_motorRC.set(speedR);
  }

  public static void setModuleDSpeed(double speedD, double speedR) {
    m_motorDD.set(speedD*N_D_SPEED);
    m_motorRD.set(speedR);
  }

  public static double getModuleRAAngle() {
    return getModuleRAPosition() >= 0 ? (getModuleRAPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 :
      (getModuleRAPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 + 1;
  }

  public static double getModuleRBAngle() {
    return getModuleRBPosition() >= 0 ? (getModuleRBPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 :
      (getModuleRBPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 + 1;
  }

  public static double getModuleRCAngle() {
    return getModuleRCPosition() >= 0 ? (getModuleRCPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 :
      (getModuleRCPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 + 1;
  }

  public static double getModuleRDAngle() {
    return getModuleRDPosition() >= 0 ? (getModuleRDPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 :
      (getModuleRDPosition() / Constants.N_MOTOR_TO_WHEEL) % 1 + 1;
  }

  public static double getModuleRAPosition() {
    return m_encoderRA.getAbsolutePosition()/360;
  }

  public static double getModuleRBPosition() {
    return m_encoderRB.getAbsolutePosition()/360;
  }

  public static double getModuleRCPosition() {
    return m_encoderRC.getAbsolutePosition()/360;
  }

  public static double getModuleRDPosition() {
    return m_encoderRD.getAbsolutePosition()/360;
  }

  public static double getModuleDAPosition() {
    return m_encoderDA.getPosition();
  }

  public static double getModuleDBPosition() {
    return m_encoderDB.getPosition();
  }

  public static double getModuleDCPosition() {
    return m_encoderDC.getPosition();
  }

  public static double getModuleDDPosition() {
    return m_encoderDD.getPosition();
  }

  public static double getAngle() {
    return 
      Math.toRadians(
        Math.IEEEremainder(
          -m_ahrs.getAngle(),
          360.0));
  }

  public static void setPositionZero() {
    m_encoderDA.setPosition(0);
    m_encoderDB.setPosition(0);
    m_encoderDC.setPosition(0);
    m_encoderDD.setPosition(0);
    m_encoderRA.setPosition(0);
    m_encoderRB.setPosition(0);
    m_encoderRC.setPosition(0);
    m_encoderRD.setPosition(0);
  }
}
