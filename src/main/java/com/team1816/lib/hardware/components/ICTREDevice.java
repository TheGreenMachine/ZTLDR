package com.team1816.lib.hardware.components;

public interface ICTREDevice extends IPhoenix6 {
    double getDeviceReference();

    double getDeviceError();

    enum DeviceType {
        TalonFX, //Falcons and Krakens
        TalonFXS, //cims, bags, etc
        Pigeon2,
        CANdle,
        CANdi,
        CANifier,
        CANrange,
        CANcoder
    }
}
