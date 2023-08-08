package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ModuleConstants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;

public class ModuleDriveIO implements ClosedLoopIO {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table;
    NetworkTable ioTable;

    private final WPI_TalonFX driveMotor;
    private boolean inverted;
    double driveOutput;
    double velocitySetpointRadPerSec;

    //PID controller for speed
    private final PIDController m_drivePIDController = new PIDController(
            ModuleConstants.kPModuleDriveController, ModuleConstants.kIModuleDriveController,
            ModuleConstants.kDModuleDriveController);

    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 8.634, 0);

    String corners;

    public ModuleDriveIO(int motorPort, boolean inverted, String corners) {
        table = inst.getTable(corners);
        ioTable = table.getSubTable("Drive");
        this.inverted = inverted;
        driveMotor = new WPI_TalonFX(motorPort, "GertrudeGreyser");
        // set status frame period of drive motor
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        driveMotor.configAllSettings(new BaseTalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralMode.Brake);
        this.corners = corners;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.appliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.velocityRadPerSec = getVelocityRadPerSecond();
        inputs.toLog(ioTable);
    }

    double getPositionRad() {
        return Units.rotationsToRadians(
                driveMotor.getSelectedSensorPosition() / ModuleConstants.kDriveEncoderTicksPerRevolution);
    }

    private double getVelocityRadPerSecond() {
        return Units.rotationsToRadians(driveMotor.getSelectedSensorVelocity()
                / ModuleConstants.kDriveEncoderTicksPerRevolution * 10);
    }

    public void setVelocityRadPerSec(double desiredSpeedRadPerSecond) {
        velocitySetpointRadPerSec = desiredSpeedRadPerSecond;
        driveOutput = desiredSpeedRadPerSecond / 8.6 //Feedforward value, without using the command
                + (
                inverted ?
                m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond)
                : -1*m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond));
        // Logger.getInstance().recordOutput(corners+" Error", getVelocityRadPerSecond()-desiredSpeedRadPerSecond);
        // Logger.getInstance().recordOutput(corners+" Setpoint", desiredSpeedRadPerSecond);
        // Logger.getInstance().recordOutput(corners+" Measurement", getVelocityRadPerSecond());

        driveMotor.setVoltage(driveOutput);
    }

 //Set Neutral Mode
    public void setBrakeMode(boolean enable) {
        if (enable) {
            driveMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

}
