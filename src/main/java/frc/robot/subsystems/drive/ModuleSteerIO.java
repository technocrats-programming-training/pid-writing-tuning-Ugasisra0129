package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// Code Needed Here!!

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;
import lib.util.TunableNumber;

public class ModuleSteerIO implements ClosedLoopIO {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table;
    NetworkTable ioTable;

    double delta;
    double deltaMod;
    double currentPos;
    double setpoint;
    double desiredRot;
    double toBeModuled;

    // Code Needed Here - Make initial object for motor.
    AnalogEncoder encoder;
    Rotation2d offset;
    double tempOffset;
    double positionSetPointRad;
    TunableNumber kP = new TunableNumber("KPTuningController", 0);
    TunableNumber kI = new TunableNumber("KITuningController", 0);
    TunableNumber kD = new TunableNumber("KDTuningController", 0);

    private PIDController m_canCoderSteeringPIDController = new PIDController(
            kP.get(), 
            kI.get(),
            kD.get());

    public ModuleSteerIO(int motorPort, int encoderPort, double encoderOffset, String corners) {
        table = inst.getTable(corners);
        ioTable = table.getSubTable("Steer");
        // Code Needed Here - Define motor
        steerMotor.configAllSettings(new BaseTalonFXConfiguration());
        steerMotor.setNeutralMode(NeutralMode.Brake);
        // set status frame period of steer motor
        steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        m_canCoderSteeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
        encoder = new AnalogEncoder(encoderPort);
        offset = new Rotation2d(encoderOffset);

        //Configure Max & Min outputs of Falcon
        steerMotor.configNominalOutputForward(0);
        steerMotor.configNominalOutputReverse(0);
        steerMotor.configPeakOutputForward(1);
        steerMotor.configPeakOutputReverse(-1);
        steerMotor.configAllowableClosedloopError(0, Constants.ModuleConstants.MaxAllowableError);
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.positionSetpointRad = setpoint;
        inputs.positionRad = getPosition().getRadians();
        inputs.appliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.toLog(ioTable);

        //Configure PID Values for built in PID on falcon
        steerMotor.setP(ioTable.getEntry("kP").getDouble(0.0));
        steerMotor.setI(ioTable.getEntry("kI").getDouble(0.0));
        steerMotor.setD(ioTable.getEntry("kD").getDouble(0.0));

        m_canCoderSteeringPIDController.setP(Constants.ModuleConstants.kPModuleTurningController);
        m_canCoderSteeringPIDController.setI(Constants.ModuleConstants.kIModuleTurningController);
        m_canCoderSteeringPIDController.setD(Constants.ModuleConstants.kDModuleTurningController);


    }

    //Get position using encoder on Falcon
    Rotation2d getPosition() {
        return Rotation2d.fromDegrees((steerMotor.getSelectedSensorPosition() /
                Constants.ModuleConstants.kSteerEncoderTicksPerRevolution) * 360);
    }

    Rotation2d getAbsPosition() {
        return Rotation2d.fromRadians(getPosition().getRadians() % (2 * Math.PI));
    }

    //Get position using CanCoder
    public Rotation2d getCanCoderPosition() {
        return Rotation2d.fromRadians(encoder.getAbsolutePosition()).minus(offset);
    }

    // Set sensor rot to cancoder position*Ticks per rev of encoder
    public void calculateOffset() {
        steerMotor.setSelectedSensorPosition(Constants.ModuleConstants.kSteerEncoderTicksPerRevolution*getCanCoderPosition().getRadians());
    }

    public double fixRot(Rotation2d position) {
        currentPos = getPosition().getRadians();
        
        // Calculate actual angle
        //delta = currentPos % (2 * Math.PI);

        toBeModuled = (position.getRadians() - (currentPos) + Math.PI/2);
        
        if (toBeModuled < 0) {
            delta = Math.PI + (toBeModuled % (Math.PI)) - (Math.PI/2);
        } else {  
            delta = (toBeModuled % (Math.PI)) - (Math.PI/2);
        }
        
        desiredRot = currentPos + delta;

        //Return final desired angle
        return desiredRot;
    }

    //Sets position using built in PID on motor
    public void setPosition(Rotation2d positionRad) {
        setpoint = fixRot(positionRad);
        // Code Needed Here!!
    }

    //Sets position using CanCoderPID
    public void setPositionCanCoder(Rotation2d positionRad) {
        positionSetPointRad = positionRad.getRadians();
        final double turnOutput = m_canCoderSteeringPIDController.calculate(
                getCanCoderPosition().getRadians(),
                positionRad.getRadians()
        );

        steerMotor.setVoltage(turnOutput);
    }
}
