// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a
   * parameter. The device will be automatically initialized with default
   * parameters.
   */
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(70);

    colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
        ProximitySensorMeasurementRate.kProxRate6ms);

  }

  public void runFwd() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.set(IntakeConstants.kIntakeMotorRateFwd);
  }

  public void runRev() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.set(-IntakeConstants.kIntakeMotorRateRev);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public double getIR() {
    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = colorSensor.getIR();
    return IR;
  }

  public int getProximity() {
    /**
     * In addition to RGB IR values, the color sensor can also return an
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    int proximity = colorSensor.getProximity();
    return proximity;
  }

  public boolean isPass() {
    return getProximity() > IntakeConstants.kIntakeGateValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IR", getIR());
    SmartDashboard.putNumber("Proximity", getProximity());
    SmartDashboard.putBoolean("isPass", isPass());
    SmartDashboard.putBoolean("isConnected", colorSensor.isConnected());

    SmartDashboard.putNumber("Intake C", intakeMotor.getOutputCurrent());
  }
}
