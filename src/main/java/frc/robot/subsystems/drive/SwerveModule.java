// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import lib.iotemplates.ClosedLoopIO;

public class SwerveModule {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table;

  String corners;

  private ModuleSteerIO steerIO;
  private ModuleDriveIO driveIO;

  private SwerveModuleState desiredState;

  private ClosedLoopIO.ClosedLoopIOInputs steerInputs;
  private ClosedLoopIO.ClosedLoopIOInputs driveInputs;

  private double offset;

  public SwerveModule(int driveMotorPort,
  int turningMotorPort, int driveEncoderPort,
   int turningEncoderPort, boolean driveEncoderReversed,
    boolean turningEncoderReversed, String corners, double turningEncoderOffset){

    table = inst.getTable(corners);

    steerIO = new ModuleSteerIO(turningMotorPort, turningEncoderPort, turningEncoderOffset, corners);
    driveIO = new ModuleDriveIO(driveMotorPort, false, corners);
    steerInputs = new ClosedLoopIO.ClosedLoopIOInputs(1);
    driveInputs = new ClosedLoopIO.ClosedLoopIOInputs(1);
    desiredState = new SwerveModuleState(0, new Rotation2d(0));
    this.corners = corners;
    this.offset = turningEncoderOffset;
    
  }

  public void periodic() {
    steerIO.updateInputs(steerInputs);
    driveIO.updateInputs(driveInputs);

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    steerIO.setPosition(desiredState.angle);
    driveIO.setVelocityRadPerSec(desiredState.speedMetersPerSecond / (Constants.DriveConstants.kWheelHeight / 2));

    table.getEntry("MagEncoder Value").setDouble(steerIO.getCanCoderPosition().getRadians());

  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
            (driveInputs.velocityRadPerSec * (Constants.DriveConstants.kWheelHeight / 2)),
            new Rotation2d(steerInputs.positionRad));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveIO.getPositionRad() / (2 * Math.PI) * Constants.DriveConstants.kMetersPerRevolution,
      steerIO.getPosition()
    );
  }
  //

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (desiredState.speedMetersPerSecond == 0) {
      this.desiredState = new SwerveModuleState(0, this.desiredState.angle);
    } else {
      this.desiredState = desiredState;
    }
  }

  /**
   * Sets the desired state for the module, even if speed is 0.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDefenseState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;
  }

  //Rotates module to 0 radians(defaut)

  public void setBrakeMode(Boolean brakeBoolean) {
    driveIO.setBrakeMode(true);
  }

  //Sets offset for this power cycle - needed to use built in encoder for PID
  public void calculateOffset() {
  steerIO.calculateOffset();
  }
}
