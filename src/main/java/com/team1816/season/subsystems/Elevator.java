package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Elevator extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "elevator";
    private final IMotor elevatorMotor = (IMotor)factory.getDevice(NAME, "elevatorMotor");
    private double curPosition;
    private ELEVATOR_STATE wantedState = ELEVATOR_STATE.FEEDER;
    VelocityVoltage elevReq = new VelocityVoltage(0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        curPosition = elevatorMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case L4 -> elevatorMotor.setControl(elevReq.withVelocity(1));
        }
    }

    public enum ELEVATOR_STATE {
        FEEDER,
        L2_CORAL,
        L3_CORAL,
        L4,
        L2_ALGAE,
        L3_ALGAE
    }

    public void setWantedState(ELEVATOR_STATE state) {
        wantedState = state;
    }
}
