package com.team1816.lib.hardware.components;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

public class GhostDevice implements IPhoenix6 {

    private final int id = 0;

    public GhostDevice(int deviceID, CANBus canbus) {

    }

    @Override
    public StatusCode setControl(ControlRequest request) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        return StatusCode.OK;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return StatusCode.OK;
    }
}
