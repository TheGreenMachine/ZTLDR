package com.team1816.lib.hardware.components.Servo;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.revrobotics.servohub.ServoHub;
import com.team1816.lib.hardware.components.IPhoenix6;

public class RevServoHubImpl extends ServoHub implements IPhoenix6 {
    /**
     * Create a new object to control a ServoHub Servo Controller
     *
     * @param deviceId The device ID.
     * @param canbus
     */
    public RevServoHubImpl(int deviceId, CANBus canbus) {
        super(deviceId);
    }

    @Override
    public StatusCode setControl(ControlRequest request) {
        return null;
    }

    @Override
    public StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails) {
        return null;
    }

    @Override
    public boolean isConnected() {
        return false;
    }

    @Override
    public StatusCode setSimSupplyVoltage(double volts) {
        return null;
    }
}
