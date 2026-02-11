package com.team1816.lib.hardware.components.sensor;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;

public class CANdiImpl extends CANdi implements IPhoenix6 {

    private CANdiSimState simState;

    public CANdiImpl(Integer canID, CANBus canBusName) {

        super(canID, canBusName);
        if(Robot.isSimulation()){
            simState = getSimState();
        }
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logging) {
        GreenLogger.periodicLog(logPath + "Connected", this::isConnected);
        return this.getConfigurator().apply((CANdiConfiguration) config);
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return simState.setSupplyVoltage(volts);
    }

}
