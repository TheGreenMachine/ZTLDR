package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team1816.lib.hardware.components.ICTREDevice;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonFXImpl extends TalonFX implements ICTREDevice, IMotor {

    private TalonFXConfiguration config;
    private TalonFXSimState simState;
    private DCMotorSim motorSim;
    private double lastSimTime;
    private final double gearing = 250.0/16.0;

    public TalonFXImpl(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        this.config = (TalonFXConfiguration) config;
        // add default logging for target and actual position/velocities
        if (logDetails) {
            GreenLogger.periodicLog(logPath + "Actual Position (rotations)", this::getMotorPosition);
            GreenLogger.periodicLog(logPath + "Actual Velocity (rps)", this::getMotorVelocity);
        }
        GreenLogger.periodicLog(logPath + "Reference", this::getDeviceReference);
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        GreenLogger.periodicLog(logPath + "Stator", ()-> getStatorCurrent().getValueAsDouble());
        if(Robot.isSimulation()){
            simState = getSimState();
            var clockwise = ((TalonFXConfiguration) config).MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
            simState.Orientation = clockwise ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
            double inertia = 0.01;
            motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, inertia, gearing), gearbox);
            lastSimTime = Utils.getCurrentTimeSeconds();
        }
        return this.getConfigurator().apply(this.config);
    }

    @Override
    public void updateSimState() {
        final double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - lastSimTime;
        lastSimTime = currentTime;

        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double frictionVoltage = 0.2;
        motorSim.setInputVoltage(addFriction(simState.getMotorVoltage(), frictionVoltage));
        motorSim.update(deltaTime);

        simState.setRawRotorPosition(motorSim.getAngularPositionRotations() * gearing);
        simState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0 * gearing);
    }

    @Override
    public double getDeviceError() {
        return getClosedLoopError().getValueAsDouble();
    }

    @Override
    public double getDeviceReference() {
        return getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return getVelocity().getValueAsDouble();
    }

    @Override
    public double getMotorPosition() {  return getPosition().getValueAsDouble(); }

    @Override
    public boolean hasDeviceCrashed() { return getStickyFault_BootDuringEnable().getValue(); }

    @Override
    public void zeroMotorPosition() {
        setPosition(0);
    }

    @Override
    public StatusCode setSimRotorVelocity(double rps) {
        return simState.setRotorVelocity(rps);
    }

    @Override
    public StatusCode setSimRotorPosition(double rotations) {
        return simState.setRawRotorPosition(rotations);
    }

    @Override
    public double getSimMotorVoltage() {
        return simState.getMotorVoltage();
    }

    @Override
    public boolean isGhost() {
        return false;
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }

    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor.
     * @param frictionVoltage Voltage required to overcome friction.
     * @return Friction-dampened motor voltage.
     */
    private double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}

