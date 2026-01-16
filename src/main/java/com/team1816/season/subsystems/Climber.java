package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private final IMotor climberMotor = (IMotor)factory.getDevice(NAME, "climberMotor");
    private double curPosition;
    private CLIMBER_STATE wantedState = CLIMBER_STATE.REST;
    VelocityVoltage climReq = new VelocityVoltage(0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        curPosition = climberMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case REST -> climberMotor.setControl(climReq.withVelocity(1));
            case L1 -> climberMotor.setControl(climReq.withVelocity(1));
            case L2 -> climberMotor.setControl(climReq.withVelocity(1));
            case L3 -> climberMotor.setControl(climReq.withVelocity(1));

        }
    }

    public enum CLIMBER_STATE {
        REST,
        L1,
        L2,
        L3
    }

    public void setWantedState(CLIMBER_STATE state) {
        wantedState = state;
    }
}
