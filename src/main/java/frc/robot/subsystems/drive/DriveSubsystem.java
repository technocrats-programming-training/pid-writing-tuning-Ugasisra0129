// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("Drive");
  //rTrigger is used to add an increased max speed relative to how much the trigger is pushed. This makes it controllable and smooth.
  double rTrigger;

  //negate to change front of robot
  private int negate = 1;
  boolean fieldOriented = false;
  // Define robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          "FrontLeft", DriveConstants.kFrontLeftOffset);


  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftDriveEncoderPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          "RearLeft", DriveConstants.kRearLeftOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          "FrontRight", DriveConstants.kFrontRightOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightDriveEncoderPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          "RearRight", DriveConstants.kRearRightOffset);

  //PID controller for rotation of robot
  ProfiledPIDController thetaController =
        new ProfiledPIDController(
            DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerConstraints);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

  //field
  private final Field2d m_field = new Field2d();

  //Slew Rate Limiters to limit speed of ditrection changes
  private final SlewRateLimiter xSpeedFilter = new SlewRateLimiter(5);
  private final SlewRateLimiter ySpeedFilter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotFilter = new SlewRateLimiter(25);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = 
     new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()});

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    calculateOffset();
    SmartDashboard.putNumber("PID P Gain Input", 0);
    SmartDashboard.putData("field", m_field);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      getHeading(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()});
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    m_field.setRobotPose(getPose()); // Be sure to use Degrees and not Radians in AdvantageScope
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
   public Pose2d getPose() {
     return m_odometry.getPoseMeters();
   }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
   public void resetOdometry(Pose2d pose) {
     m_odometry.resetPosition(getHeading(), 
     new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()},
     pose);
   }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param rTrigger right trigger for boosted speed
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, double rTrigger) {

    rot =  -1* MathUtil.applyDeadband(rot, 0.4);
    ySpeed = negate*MathUtil.applyDeadband(ySpeed, 0.2);
    xSpeed =  negate*MathUtil.applyDeadband(xSpeed, 0.2);

    if (Math.abs(xSpeed) > 0.2) {
      xSpeed = xSpeed + (xSpeed/Math.abs(xSpeed)) * rTrigger * DriveConstants.kBoostModifier;
    } 

    if (Math.abs(ySpeed) > 0.2) {
      ySpeed = ySpeed + (ySpeed/Math.abs(ySpeed)) * rTrigger * DriveConstants.kBoostModifier;
    }
    
    rot = rotFilter.calculate(rot);
    ySpeed = ySpeedFilter.calculate(ySpeed);
    xSpeed = xSpeedFilter.calculate(xSpeed);
    this.rTrigger = rTrigger;

    driveNoDeadband(xSpeed, ySpeed, rot);
  }

  public void driveNoDeadband(double xSpeed, double ySpeed, double rot) {

    SwerveModuleState[] swerveModuleStates;
    if (fieldOriented) {
      swerveModuleStates =
              DriveConstants.kDriveKinematics.toSwerveModuleStates(
                      // fieldRelative
                      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
    } else{
      swerveModuleStates =
              DriveConstants.kDriveKinematics.toSwerveModuleStates(
                      new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

        //Desaturate = make sure speed for each module is achievable(Used to be called normalize)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond + rTrigger * (DriveConstants.kBoostModifier));
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_rearLeft.setDesiredState(swerveModuleStates[1]);
    m_frontRight.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
        //Desaturate = make sure speed for each module is achievable(Used to be called normalize)
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond + rTrigger * (DriveConstants.kBoostModifier));
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[1]);
    m_frontRight.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  //Sets proper offset for encoders
  public void calculateOffset() {
    m_frontLeft.calculateOffset();
    m_frontRight.calculateOffset();
    m_rearLeft.calculateOffset();
    m_rearRight.calculateOffset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1 * m_gyro.getFusedHeading());
  }

  public void autoRotate(double xSpeed, double ySpeed, double desiredAngleRad, double rTrigger) {
    drive(xSpeed, ySpeed, thetaController.calculate(getHeading().getRadians(), desiredAngleRad), rTrigger);
  }

  public void fieldON() {
    fieldOriented = true;
    driveNoDeadband(0, 0, 0);
  }

  public void fieldOFF() {
    fieldOriented = false;
    driveNoDeadband(0, 0, 0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  //Rotates all modules to point to center
  public void defence() {
    m_frontLeft.setDefenseState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
    m_frontRight.setDefenseState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
    m_rearLeft.setDefenseState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
    m_rearRight.setDefenseState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
  }

  //Changes brake mode of all modules
  public void setBrakeMode(Boolean brakeBoolean) {
    m_frontLeft.setBrakeMode(brakeBoolean);
    m_frontRight.setBrakeMode(brakeBoolean);
    m_rearLeft.setBrakeMode(brakeBoolean);
    m_rearRight.setBrakeMode(brakeBoolean);
  }

  public float getRoll() {
    return -1* m_gyro.getRoll();
  }

  public float getPitch() {
    return m_gyro.getPitch();
  }

  public float getYaw() {
    return m_gyro.getYaw();
  }

  //Flips front of robot
  public void makeBackwards(boolean GOGOGOGOGOGOGO) {
    if (GOGOGOGOGOGOGO){
      negate = -1;
    } else{
      negate = 1;
    }
  }

  public void balance() {
    if (m_gyro.getPitch() < -AutoConstants.kBalanceDeadzoneAngle) {
      drive(AutoConstants.kBalanceDriveSpeed, 0, 0, 0);
    } else if (m_gyro.getPitch() > AutoConstants.kBalanceDeadzoneAngle) {
      drive(-AutoConstants.kBalanceDriveSpeed, 0, 0, 0);
    } else {
      //it's flat
      defence();
    }
  }

}
