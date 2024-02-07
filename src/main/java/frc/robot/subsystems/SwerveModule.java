// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class SwerveModule extends SubsystemBase {

  // Create empty variables for reassignment
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private SparkPIDController builtinTurningPidController;

  // private final DutyCycleEncoder absoluteEncoder;
  private final CANcoder absoluteEncoder;

  private String moduleName;

  // Special UI variables for swerve simulation
  private MechanismLigament2d simTurnCmd;
  private MechanismLigament2d simDirectionCmd;

  private MechanismLigament2d simTurnReal;
  private MechanismLigament2d simDirectionReal;

  // Class constructor where we assign default values for variable
  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      int absoluteEncoderId,
      String name) {

    moduleName = name;

    // Create absolute encoder
    absoluteEncoder = new CANcoder(absoluteEncoderId);

    // Create drive and turning motor
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // Set reverse state of drive and turning motor
    driveMotor.setInverted(true);
    turningMotor.setInverted(true);

    // Set drive and turning motor encoder values
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    // Change drive motor conversion factors
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Change conversion factors for neo turning encoder - should be in radians!
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // -----SPARK-MAX-PID-----//
    builtinTurningPidController = turningMotor.getPIDController();

    // Set PID values for the simulated Spark max PID
    builtinTurningPidController.setPositionPIDWrappingEnabled(true);
    builtinTurningPidController.setPositionPIDWrappingMinInput(-Math.PI);
    builtinTurningPidController.setPositionPIDWrappingMaxInput(Math.PI);
    builtinTurningPidController.setP(ModuleConstants.kPTurning);
    builtinTurningPidController.setI(ModuleConstants.kITurning);
    builtinTurningPidController.setD(ModuleConstants.kDTurning);
    builtinTurningPidController.setIZone(0.0);
    builtinTurningPidController.setFF(0.0);
    builtinTurningPidController.setOutputRange(-1, 1);
    turningMotor.burnFlash();

    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setSmartCurrentLimit(40);
    turningMotor.setSmartCurrentLimit(20);

    // Call resetEncoders
    resetEncoders();

    // Thanks to Alec for this code!
    // >-----------S-I-M------------<//

    // Create the mechanism 2d canvas and get the root
    Mechanism2d mod = new Mechanism2d(6, 6);
    MechanismRoot2d root = mod.getRoot("root", 3, 3);

    // Add simTurn to the root, add direction to turn, then add it to smart
    // dashboard
    simTurnCmd = root.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirectionCmd = simTurnCmd
        .append(new MechanismLigament2d("Wheel Speed", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName + " commanded Turn", mod);

    // ------------//

    // Do the same thing but for the real module state
    Mechanism2d mod2 = new Mechanism2d(6, 6);
    MechanismRoot2d root2 = mod2.getRoot("root", 3, 3);

    simTurnReal = root2.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirectionReal = simTurnReal
        .append(new MechanismLigament2d("Wheel Speed", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName + "  real Turn", mod2);

    // >-------------------------------<//

  }

  public void update() {
    // SmartDashboard.putNumber(moduleName + "Absolute-Position", getAbsoluteEncoderRad());
  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return (new SwerveModulePosition(
        getDrivePosition(), new Rotation2d(getTurningPosition())));
  }

  /*
   * Convert absolute value of the encoder to radians and then subtract the radian
   * offset
   * then check if the encoder is reversed.
   */
  public double getAbsoluteEncoderRad() {
    return absoluteEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    // Check if new command has high driving power
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never
    // rotate more than 90*
    state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(
        state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kMotorMaxOutput);

    // Use PID to calculate angle setpoint
    builtinTurningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);

    simTurnCmd.setAngle(state.angle); // .plus(Rotation2d.fromDegrees(90))
    simDirectionCmd.setAngle(state.speedMetersPerSecond > 0 ? 0 : 180);
    simDirectionCmd.setLength(Math.abs(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

    simTurnReal.setAngle(absoluteEncoder.getAbsolutePosition().getValue() * 360); // +90
    simDirectionReal
        .setAngle(getDriveVelocity() > 0 ? 0 : 180);
    simDirectionReal.setLength(Math.abs(getDriveVelocity() / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));

    SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
  }

  // Stop all motors on module
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // Motor and SparkMax methods for Monitor
  public double[] getMotorsCurrent() {
    return (new double[] { driveMotor.getOutputCurrent(), turningMotor.getOutputCurrent() });
  }

  public double[] getMotorsTemp() {
    return (new double[] { driveMotor.getMotorTemperature(), turningMotor.getMotorTemperature() });
  }

  public void setSmartCurrentLimiter(int driveLimit, int turningLimit) {
    driveMotor.setSmartCurrentLimit(driveLimit);
    turningMotor.setSmartCurrentLimit(driveLimit);
  }

}
