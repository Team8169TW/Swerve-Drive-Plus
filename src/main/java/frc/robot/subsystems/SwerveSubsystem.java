// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      "Front Left");

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      "Front Right");

  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      "Back Left");

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      "Back Right");

  // The end of this madness ^_^

  // Create the navX using roboRIO expansion port
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private Field2d field = new Field2d();

  private PIDController thetaControllerN;
  private PIDController thetaControllerA;

  private double heading;

  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions() {

    return (new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition() });

  }

  // Create odometer for swerve drive
  private SwerveDriveOdometry odometer;

  // Swerve subsystem constructor
  public SwerveSubsystem() {

    // Reset robot encoders on startup
    resetAllEncoders();

    // Zero navX heading on new thread when robot starts
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        // gyro.calibrate();
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    // Set robot odometry object to current robot heading and swerve module
    // positions
    odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
        getOdometryAngle(), getModulePositions());
    // new Rotation2d(gyro.getYaw() * -1 / 180 * Math.PI), getModulePositions()

    // Set default PID values for thetaPID
    thetaControllerN = new PIDController(DriveConstants.kPThetaN, DriveConstants.kIThetaN,
        DriveConstants.kDThetaN);
    thetaControllerA = new PIDController(DriveConstants.kPThetaA, DriveConstants.kIThetaA,
        DriveConstants.kDThetaA);

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::setChassisSpeeds,
        AutoConstants.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  // Reset gyro heading
  public void zeroHeading() {
    gyro.reset();
    heading = getHeading();
  }

  // // Reset gyro yaw
  // public void resetYaw() {
  // gyro.zeroYaw();
  // }

  // public void calibrateGyro(){
  // gyro.calibrate();
  // }

  // Return gyro heading, make sure to read navx docs on this
  public double getHeading() {
    return gyro.getAngle();
  }

  // Return the robot odometry in pose meters
  // public Pose2d getOdometryMeters() {
  // return (odometer.getPoseMeters());
  // }

  // Return heading in Rotation2d format
  // public Rotation2d getRotation2d() {
  // return Rotation2d.fromDegrees(getHeading());
  // }

  // Stop all module movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  // Move the swerve modules to the desired SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module
    // roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set the module state
    setModuleStates(moduleStates);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle) {
    setChassisOutput(xSpeed, ySpeed, turningAngle, false, false);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle, boolean forAuto) {
    setChassisOutput(xSpeed, ySpeed, turningAngle, forAuto, false);
  }

  public void setChassisOutput(double xSpeed, double ySpeed, double turningAngle, boolean forAuto,
      boolean robotRelative) {
    xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    double turningSpeed;
    if (forAuto) {
      heading = getHeading() - turningAngle;
      turningSpeed = thetaControllerA.calculate(getHeading(), heading);
    } else {
      heading -= turningAngle;
      turningSpeed = thetaControllerN.calculate(getHeading(), heading);
    }

    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
    turningSpeed *= -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    turningSpeed = MathUtil.clamp(turningSpeed, -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
        DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    if (robotRelative) {
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
          Rotation2d.fromDegrees(getRobotDegrees()));
    } else {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
          Rotation2d.fromDegrees(getRobotDegrees()));
    }

    // Set chassis speeds
    setChassisSpeeds(chassisSpeeds);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();
    return states;
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // Return robot position caculated by odometer
  public Pose2d getPose() {
    return odometer.getPoseMeters();
    // return odometer.getPoseMeters().rotateBy(Rotation2d.fromDegrees(-90));
    // return odometer.getPoseMeters().transformBy(new Transform2d(0, 0,
    // Rotation2d.fromDegrees(-90)));
  }

  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }

  // Reset odometer to new Pose2d location but with roation
  // public void resetOdometry(Pose2d pose, Rotation2d rot) {
  // odometer.resetPosition(rot, getModulePositions(), pose);
  // }

  // Return an angle from -180 to 180 for robot odometry
  // The commented out method is for if the gyroscope is reversed direction
  public Rotation2d getOdometryAngle() {
    /*
     * double angle = -gyro.getYaw() + 180;
     * if(angle > 180){
     * angle -= 360;
     * }else if(angle < -180){
     * angle += 360;
     * }
     * return Rotation2d.fromDegrees(angle);
     */
    // SmartDashboard.putNumber("Yaw", gyro.getYaw());
    // SmartDashboard.putNumber("Angle", gyro.getAngle());
    // return (Rotation2d.fromDegrees(gyro.getYaw()));
    return Rotation2d.fromDegrees(getRobotDegrees() - 180);
  }

  // Returns an angle from 0 to 360 that is continuous, meaning it loops
  public double getRobotDegrees() {
    double rawValue = -gyro.getAngle() % 360.0;
    if (rawValue < 0.0) {
      return (rawValue + 360.0);
    } else {
      return (rawValue);
    }
  }

  // Reset all swerve module encoders
  public void resetAllEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  // public double getRumble() {
  // return gyro.getRawAccelX();
  // }

  // public double getRollChange() {
  // return (gyro.getRawGyroY());
  // }

  // public double getRoll() {
  // return (gyro.getRoll());
  // }

  // public double getRobotForceNewtons() {
  // return (57.0 * 9.8 * gyro.getRawAccelX());
  // }

  // Periodic looooooop
  @Override
  public void periodic() {

    // Update odometer for it to caculate robot position
    odometer.update(getOdometryAngle(), getModulePositions());

    field.setRobotPose(getPose());

    // Put odometry data on smartdashboard
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("ROBOT DEGREES NAVX", getRobotDegrees());
    SmartDashboard.putString("ODOMETRY", odometer.getPoseMeters().toString());
    SmartDashboard.putString("Raw R2d ROBOT DEG", getOdometryAngle().toString());

    SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
    SmartDashboard.putBoolean("Magnetic Issues", gyro.isMagneticDisturbance());
    SmartDashboard.putBoolean("Magnetic Calibartion", gyro.isMagnetometerCalibrated());

    SmartDashboard.putNumber("Robot Acceleration X", gyro.getRawAccelX());
    SmartDashboard.putNumber("Robot Acceleration Y", gyro.getRawAccelY());
    SmartDashboard.putNumber("Robot Force X Newtons", 57.0 * 9.8 * gyro.getRawAccelX());
    SmartDashboard.putNumber("Robot Force X Pounds", (57.0 * 9.8 * gyro.getRawAccelX()) / 4.45);

    // SmartDashboard.putNumber("RAW ROLL", getRoll());
    // SmartDashboard.putNumber("RAW Y", getRollChange());

    // Update smartdashboard data for each swerve module object
    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();

  }
}