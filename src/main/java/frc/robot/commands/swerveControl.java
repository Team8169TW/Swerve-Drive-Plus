// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class swerveControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static DriveTrain m_driveTrain;
  private DoubleSupplier m_xbox_LX;
  private DoubleSupplier m_xbox_LY;
  private DoubleSupplier m_xbox_RX;

  /** 
   * @param subsystem The subsystem used by this command.
   */
  public swerveControl(DriveTrain subsystem, DoubleSupplier xbox_LX, DoubleSupplier xbox_LY, DoubleSupplier xbox_RX) {
    m_driveTrain = subsystem;
    m_xbox_LX = xbox_LX;
    m_xbox_LY = xbox_LY;
    m_xbox_RX = xbox_RX;
    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    String sMode = SmartDashboard.getString("S-Mode", "default");
    double forward, strafe, azimuth;
    System.out.println(sMode);
    if(sMode.equals("smart")){
      double sForward = SmartDashboard.getNumber("S-Forward", -100);
      double sStrafe = SmartDashboard.getNumber("S-Strafe", -100);
      double sAzimuth = SmartDashboard.getNumber("S-Azimuth", -100);
      System.out.println(sMode+" "+sForward+" "+sStrafe+" "+sAzimuth);
      if(sForward != -100 && sStrafe != -100 && sAzimuth != -100){
        forward = sForward;
        strafe = sStrafe;
        azimuth = sAzimuth;
      }else{
        forward = -m_xbox_LY.getAsDouble();
        strafe = -m_xbox_LX.getAsDouble();
        azimuth = -m_xbox_RX.getAsDouble();
      }
    }else{
      forward = -m_xbox_LY.getAsDouble();
      strafe = -m_xbox_LX.getAsDouble();
      azimuth = -m_xbox_RX.getAsDouble();
    }
    double angle = m_driveTrain.getAngle();
    double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
    strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
    forward = temp;
    double a = strafe - azimuth;
    double b = strafe + azimuth;
    double c = forward - azimuth;
    double d = forward + azimuth;
    double sDB = Math.hypot(b, c);
    double sDC = Math.hypot(b, d);
    double sDD = Math.hypot(a, d);
    double sDA = Math.hypot(a, c);
    double maxWheelSpeed = Math.max(Math.max(sDA, sDB), Math.max(sDC, sDD));
    double angle1 = maxWheelSpeed > 0.05 ? Math.atan2(a, c) * 0.5 / Math.PI : 0.625;
    double angle2 = maxWheelSpeed > 0.05 ? Math.atan2(b, c) * 0.5 / Math.PI : 0.375;
    double angle3 = maxWheelSpeed > 0.05 ? Math.atan2(b, d) * 0.5 / Math.PI : 0.125;
    double angle4 = maxWheelSpeed > 0.05 ? Math.atan2(a, d) * 0.5 / Math.PI : 0.875;
    double sRA = angle1 - m_driveTrain.getModuleRAAngle() < -0.5 ? (angle1 - m_driveTrain.getModuleRAAngle() + 1) : angle1 - m_driveTrain.getModuleRAAngle() > 0.5 ? angle1 - m_driveTrain.getModuleRAAngle() - 1 : angle1 - m_driveTrain.getModuleRAAngle();
    double sRB = angle2 - m_driveTrain.getModuleRBAngle() < -0.5 ? (angle2 - m_driveTrain.getModuleRBAngle() + 1) : angle2 - m_driveTrain.getModuleRBAngle() > 0.5 ? angle2 - m_driveTrain.getModuleRBAngle() - 1 : angle2 - m_driveTrain.getModuleRBAngle();
    double sRC = angle3 - m_driveTrain.getModuleRCAngle() < -0.5 ? (angle3 - m_driveTrain.getModuleRCAngle() + 1) : angle3 - m_driveTrain.getModuleRCAngle() > 0.5 ? angle3 - m_driveTrain.getModuleRCAngle() - 1 : angle3 - m_driveTrain.getModuleRCAngle();
    double sRD = angle4 - m_driveTrain.getModuleRDAngle() < -0.5 ? (angle4 - m_driveTrain.getModuleRDAngle() + 1) : angle4 - m_driveTrain.getModuleRDAngle() > 0.5 ? angle4 - m_driveTrain.getModuleRDAngle() - 1 : angle4 - m_driveTrain.getModuleRDAngle();
    if (maxWheelSpeed > 1.0) {
      sDA /= maxWheelSpeed;
      sDB /= maxWheelSpeed;
      sDC /= maxWheelSpeed;
      sDD /= maxWheelSpeed;
    }
    if (maxWheelSpeed > 0.05) {
      m_driveTrain.setModuleASpeed(sDA, sRA);
      m_driveTrain.setModuleBSpeed(sDB, sRB);
      m_driveTrain.setModuleCSpeed(sDC, sRC);
      m_driveTrain.setModuleDSpeed(sDD, sRD);
    } else {
      m_driveTrain.setModuleASpeed(0, sRA);
      m_driveTrain.setModuleBSpeed(0, sRB);
      m_driveTrain.setModuleCSpeed(0, sRC);
      m_driveTrain.setModuleDSpeed(0, sRD);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
