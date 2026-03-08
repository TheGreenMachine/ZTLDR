package com.team1816.lib.hardware.components;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.team1816.lib.hardware.components.motor.IMotor;

public class GhostDevice implements IMotor {

    private int Id = 0;

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

    @Override
    public double getMotorVelocity() {
        return 0;
    }

    @Override
    public double getMotorPosition() {
        return 0;
    }

    @Override
    public void zeroMotorPosition() {

    }

    @Override
    public StatusCode setSimRotorVelocity(double rps) {
        return StatusCode.OK;
    }

    @Override
    public StatusCode setSimRotorPosition(double rotations) {
        return StatusCode.OK;
    }

    @Override
    public double getSimMotorVoltage() {
        return 12;
    }

    @Override
    public boolean hasDeviceCrashed() { return  false; }
}
