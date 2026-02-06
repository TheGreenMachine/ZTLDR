package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private final IMotor climberMotor = (IMotor)factory.getDevice(NAME, "climberMotor");
    private double curPosition;
    private CLIMBER_STATE wantedState = CLIMBER_STATE.IDLING;
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
            case CLIMBING:

                break;
            case DOWNCLIMBING:

                break;
            case IDLING:
            default:
                break;
        }

        SmartDashboard.putString("Climber state: ", wantedState.toString());
    }

    public enum CLIMBER_STATE {
        CLIMBING,
        DOWNCLIMBING,
        IDLING
    }

    public void setWantedState(CLIMBER_STATE wantedState) {
        this.wantedState = wantedState;
    }
}
