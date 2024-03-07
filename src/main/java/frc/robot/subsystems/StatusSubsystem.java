// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.Constants.LinkageConstants.LinkageState;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class StatusSubsystem extends SubsystemBase {
  private static boolean swerveAutoByIntake = false;
  private static boolean swerveAutoByIntakeSig = false;
  private static boolean swerveAutoByShooter = false;
  private static boolean swerveAutoByShooterSig = false;
  private static IntakeState intakeState = IntakeState.kStop;
  private static boolean intakeStateSig = false;
  private static ShooterState shooterStateT = ShooterState.kStop;
  private static boolean shooterStateSigT = false;
  private static ShooterState shooterStateB = ShooterState.kStop;
  private static boolean shooterStateSigB = false;
  private static LinkageState linkageState = LinkageState.kOk;
  private static boolean linkageStateSig = false;

  private static boolean intakeNotePassed = false;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue = 0;
  private int ffc = 0;

  /** Creates a new StatusSubsystem. */
  public StatusSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(160);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // LED Addr:
  //    Back R: 0~16
  // Linkage R: 17~44
  // Shooter R: 45~60
  // Shooter L: 61~77
  // Linkage L: 78~105
  //    Back L: 106~122

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void setLedAll(int hue) {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
  }

  private void setLedLinkage(int hue) {
    for (var i = 17; i <= 44; i++) {
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    for (var i = 78; i <= 105; i++) {
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
  }

  private void setLedBack(int hue) {
    for (var i = 0; i <= 16; i++) {
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    for (var i = 106; i <= 122; i++) {
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
  }

  private void ledByShooterState() {
    if (shooterStateT == ShooterState.kStop && shooterStateB == ShooterState.kStop) {
      setLedLinkage(90); // cyan
      ffc = 0;
    } else if (shooterStateT == ShooterState.kPreparing || shooterStateB == ShooterState.kPreparing) {
      if (ffc % 16 > 8) {
        setLedLinkage(0); // red
      } else {
        setLedLinkage(60); // green
      }
      ffc++;
    } else {
      setLedLinkage(60); // green
    }
  }

  private void ledByNoteState(){
    if(intakeNotePassed){
      setLedBack(20); // orange
    }else{
      setLedBack(90); // cyan
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ledByShooterState();
    ledByNoteState();
    m_led.setData(m_ledBuffer);

    if (swerveAutoByIntake) {
      swerveAutoByIntakeSig = !swerveAutoByIntakeSig;
    } else {
      swerveAutoByIntakeSig = false;
    }
    SmartDashboard.putBoolean("Swerve AutoByIntake", swerveAutoByIntakeSig);

    if (swerveAutoByShooter) {
      swerveAutoByShooterSig = !swerveAutoByShooterSig;
    } else {
      swerveAutoByShooterSig = false;
    }
    SmartDashboard.putBoolean("Swerve AutoByShooter", swerveAutoByShooterSig);

    if (intakeState == IntakeState.kStop) {
      intakeStateSig = false;
    } else if (intakeState == IntakeState.kRunning) {
      intakeStateSig = true;
    } else {
      intakeStateSig = !intakeStateSig;
    }
    SmartDashboard.putBoolean("Intake state", intakeStateSig);

    if (shooterStateT == ShooterState.kStop) {
      shooterStateSigT = false;
    } else if (shooterStateT == ShooterState.kReady) {
      shooterStateSigT = true;
    } else {
      shooterStateSigT = !shooterStateSigT;
    }
    SmartDashboard.putBoolean("Shooter state T", shooterStateSigT);
    if (shooterStateB == ShooterState.kStop) {
      shooterStateSigB = false;
    } else if (shooterStateB == ShooterState.kReady) {
      shooterStateSigB = true;
    } else {
      shooterStateSigB = !shooterStateSigB;
    }
    SmartDashboard.putBoolean("Shooter state B", shooterStateSigB);

    if (linkageState == LinkageState.kOk) {
      linkageStateSig = true;
    } else {
      linkageStateSig = !linkageStateSig;
    }
    SmartDashboard.putBoolean("Linkage state", linkageStateSig);
  }

  public static void setSwerveAuto(boolean on, Limelight by) {
    if (by == Limelight.kInatke) {
      swerveAutoByIntake = on;
    } else {
      swerveAutoByShooter = on;
    }
  }

  public static void setIntake(IntakeState state) {
    intakeState = state;
  }

  public static void setSooterT(ShooterState state) {
    shooterStateT = state;
  }

  public static void setSooterB(ShooterState state) {
    shooterStateB = state;
  }

  public static void setLinkage(LinkageState state) {
    linkageState = state;
  }

  public static void setNotePassed(boolean passed){
    intakeNotePassed = passed;
  }
}
