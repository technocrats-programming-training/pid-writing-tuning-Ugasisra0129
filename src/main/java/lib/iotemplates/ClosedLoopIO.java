// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;


import edu.wpi.first.networktables.NetworkTable;

/** Template hardware interface for a closed loop subsystem. */
public interface ClosedLoopIO {
    /** Contains all of the input data received from hardware. */
    public static class ClosedLoopIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double velocitySetpointRadPerSec = 0;
        public double positionSetpointRad = 0;
        public double appliedVolts = 0.0;
        public double[] supplyCurrentAmps = new double[] {};
        public double[] statorCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
        public boolean[] underVoltage = new boolean[] {};
        public boolean[] forwardLimitSwitch = new boolean[] {};
        public boolean[] reverseLimitSwitch = new boolean[] {};
        public boolean[] forwardSoftLimit = new boolean[] {};
        public boolean[] reverseSoftLimit = new boolean[] {};
        public boolean[] hardwareFailure = new boolean[] {};
        public boolean[] resetDuringEn = new boolean[] {};
        public boolean[] sensorOverflow = new boolean[] {};
        public boolean[] sensorOutOfPhase = new boolean[] {};
        public boolean[] hardwareESDReset = new boolean[] {};
        public boolean[] remoteLossOfSignal = new boolean[] {};
        public boolean[] APIError = new boolean[] {};
        public boolean[] supplyOverV = new boolean[] {};
        public boolean[] supplyUnstable = new boolean[] {};

        public ClosedLoopIOInputs(int numMotors) {
            supplyCurrentAmps = new double[numMotors];
            statorCurrentAmps = new double[numMotors];
            tempCelcius = new double[numMotors];
            underVoltage = new boolean[numMotors];
            forwardLimitSwitch = new boolean[numMotors];
            reverseLimitSwitch = new boolean[numMotors];
            forwardSoftLimit = new boolean[numMotors];
            reverseSoftLimit = new boolean[numMotors];
            hardwareFailure = new boolean[numMotors];
            resetDuringEn = new boolean[numMotors];
            sensorOverflow = new boolean[numMotors];
            sensorOutOfPhase = new boolean[numMotors];
            hardwareESDReset = new boolean[numMotors];
            remoteLossOfSignal = new boolean[numMotors];
            APIError = new boolean[numMotors];
            supplyOverV = new boolean[numMotors];
            supplyUnstable = new boolean[numMotors];
        }

        public void toLog(NetworkTable table) {
            table.getEntry("PositionRad").setDouble(positionRad);
            table.getEntry("VelocityRadPerSec").setDouble(velocityRadPerSec);
            table.getEntry("SetpointRadPerSec").setDouble(velocitySetpointRadPerSec);
            table.getEntry("PositionSetpointRad").setDouble(positionSetpointRad);
            table.getEntry("AppliedVolts").setDouble(appliedVolts);
            table.getEntry("SupplyCurrentAmps").setDoubleArray(supplyCurrentAmps);
            table.getEntry("StatorCurrentAmps").setDoubleArray(statorCurrentAmps);
            table.getEntry("TempCelcius").setDoubleArray(tempCelcius);
            table.getEntry("UnderVoltage").setBooleanArray(underVoltage);
            table.getEntry("ForwardLimitSwitch").setBooleanArray(forwardLimitSwitch);
            table.getEntry("ReverseLimitSwitch").setBooleanArray(reverseLimitSwitch);
            table.getEntry("ForwardSoftLimit").setBooleanArray(forwardSoftLimit);
            table.getEntry("ReverseSoftLimit").setBooleanArray(reverseSoftLimit);
            table.getEntry("HardwareFailure").setBooleanArray(hardwareFailure);
            table.getEntry("ResetDuringEn").setBooleanArray(resetDuringEn);
            table.getEntry("SensorOverflow").setBooleanArray(sensorOverflow);
            table.getEntry("SensorOutOfPhase").setBooleanArray(sensorOutOfPhase);
            table.getEntry("HardwareESDReset").setBooleanArray(hardwareESDReset);
            table.getEntry("RemoteLossOfSignal").setBooleanArray(remoteLossOfSignal);
            table.getEntry("APIError").setBooleanArray(APIError);
            table.getEntry("SupplyOverV").setBooleanArray(supplyOverV);
            table.getEntry("SupplyUnstable").setBooleanArray(supplyUnstable);
        }

        public void fromLog(NetworkTable table) {
            positionRad = table.getEntry("PositionRad").getDouble(0);
            velocityRadPerSec = table.getEntry("VelocityRadPerSec").getDouble(0);
            appliedVolts = table.getEntry("AppliedVolts").getDouble(0);
            supplyCurrentAmps = table.getEntry("StatorCurrentAmps").getDoubleArray(new double[] {});
            statorCurrentAmps = table.getEntry("StatorCurrentAmps").getDoubleArray(new double[] {});
            tempCelcius = table.getEntry("TempCelcius").getDoubleArray(new double[] {});
            underVoltage = table.getEntry("UnderVoltage").getBooleanArray(new boolean[] {});
            forwardLimitSwitch = table.getEntry("ForwardLimitSwitch").getBooleanArray(new boolean[] {});
            reverseLimitSwitch = table.getEntry("ReverseLimitSwitch").getBooleanArray(new boolean[] {});
            forwardSoftLimit = table.getEntry("ForwardSoftLimit").getBooleanArray(new boolean[] {});
            reverseSoftLimit = table.getEntry("ReverseSoftLimit").getBooleanArray(new boolean[] {});
            hardwareFailure = table.getEntry("HardwareFailure").getBooleanArray(new boolean[] {});
            resetDuringEn = table.getEntry("ResetDuringEn").getBooleanArray(new boolean[] {});
            sensorOverflow = table.getEntry("SensorOverflow").getBooleanArray(new boolean[] {});
            sensorOutOfPhase = table.getEntry("SensorOutOfPhase").getBooleanArray(new boolean[] {});
            hardwareESDReset = table.getEntry("HardwareESDReset").getBooleanArray(new boolean[] {});
            remoteLossOfSignal = table.getEntry("RemoteLossOfSignal").getBooleanArray(new boolean[] {});
            APIError = table.getEntry("APIError").getBooleanArray(new boolean[] {});
            supplyOverV = table.getEntry("SupplyOverV").getBooleanArray(new boolean[] {});
            supplyUnstable = table.getEntry("SupplyUnstable").getBooleanArray(new boolean[] {});
            velocitySetpointRadPerSec = table.getEntry("SetpointRadPerSec").getDouble(0);
            positionSetpointRad = table.getEntry("PositionSetpointRad").getDouble(0);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClosedLoopIOInputs inputs) {

    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    /**
     * Run closed loop at the specified velocity.
     *
     * @param velocityRadPerSec Velocity setpoint.
     */
    public default void setVelocity(double velocityRadPerSec) {
    }

    public default void setPosition(double positionRad) {

    }

    /** Enable or disable brake mode. */
    public default void setBrakeMode(boolean enable) {
    }

    /** Set velocity PID constants. */
    public default void configurePID(double kp, double ki, double kd) {
    }

    /** Reset the encoder(s) to a known position. */
    public default void resetPosition(double positionRad) {
    }
}