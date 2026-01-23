package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private boolean wantedStateChanged;

    private CLIMBER_WANTED_STATE wantedState = CLIMBER_WANTED_STATE.STOW;

    private final IMotor climbMotor = (IMotor)factory.getDevice(NAME, "climberMotor");
    PositionVoltage climbReq = new PositionVoltage(0);
    private final IMotor stowerMotor = (IMotor)factory.getDevice(NAME, "stowMotor");
    PositionVoltage stowerReq = new PositionVoltage(0);

    private double climbCurrentPosition;
    private double stowerCurrentPosition;

    private double climbPositionStow = factory.getConstant(NAME, "climbPositionStow", 0);
    private double climbPositionReady = factory.getConstant(NAME, "climbPositionUnstow", 0);
    private double climbPositionL1 = factory.getConstant(NAME, "climbPositionL1", 0);
    private double climbPositionL2 = factory.getConstant(NAME, "climbPositionL2", 0);
    private double climbPositionL3 = factory.getConstant(NAME, "climbPositionL3", 0);

    private double stowerPositionStow = factory.getConstant(NAME, "stowerPositionStow", 0);
    private double stowerPositionUnstow = factory.getConstant(NAME, "stowerPositionUnstow", 0);

    private double climbReadyPositionTolerance = 0.01;
    private double stowerStowPositionTolerance = 0.01;

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        climbCurrentPosition = climbMotor.getMotorPosition();
        stowerCurrentPosition = stowerMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case STOW -> coordinateControlRequest(climbPositionStow, stowerPositionStow);
            case READY -> coordinateControlRequest(climbPositionReady, stowerPositionUnstow);
            case L1 -> coordinateControlRequest(climbPositionL1, stowerPositionUnstow);
            case L2 -> coordinateControlRequest(climbPositionL2, stowerPositionUnstow);
            case L3 -> coordinateControlRequest(climbPositionL3, stowerPositionUnstow);
        }
    }

    private void coordinateControlRequest(double climbMotorReq, double stowerMotorReq){
/*        if (Math.abs(climbCurrentPosition - climbPositionStow) <= climbReadyPositionTolerance){
            if (stowerReq.getControlInfo())
            stowerMotor.setControl(stowerReq.withPosition(stowerMotorReq));

        } else {
            climbMotor.setControl(climbReq.withPosition(climbMotorReq));
        }*/

        climbMotor.setControl(climbReq.withPosition(climbMotorReq));
        stowerMotor.setControl(stowerReq.withPosition(stowerMotorReq));
    }

    public enum CLIMBER_WANTED_STATE {
        STOW,
        READY,
        L1,
        L2,
        L3
    }

    public enum CLIMBER_SYSTEM_STATE {
        STOW,
        READY,
        L1,
        L2,
        L3
    }

    public void setWantedState(CLIMBER_WANTED_STATE wantedState) {
        wantedStateChanged = true;
        this.wantedState = wantedState;
    }
}
