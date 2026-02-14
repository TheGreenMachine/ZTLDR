package com.team1816.lib.hardware.components;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

/**
 * Common methods that are used to interact with the CTRE hardware.
 * They have direct API definitions in the Phoenix Library.
 */
public interface IPhoenix6 {
    /**
     * Common method for setting Control Requests.  i.e. make hardware do things
     */
    StatusCode setControl(ControlRequest request);

    /**
     * Applies configuration changes
     */
    StatusCode applyConfiguration(ParentConfiguration config, String logPath, boolean logDetails);

    /**
     * @return whether device is on the CANbus
     */
    boolean isConnected();

    StatusCode setSimSupplyVoltage(double volts);
}
