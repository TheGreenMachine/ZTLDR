package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Indexer extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "indexer";
    private final IMotor indexerMotor = (IMotor)factory.getDevice(NAME, "indexerMotor");
    private double curPosition;
    private INDEXER_STATE wantedState = INDEXER_STATE.FEEDER;
    VelocityVoltage indexReq = new VelocityVoltage(0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();

        CommandScheduler.getInstance().getDefaultButtonLoop().poll();
    }

    @Override
    public void readFromHardware() {
        curPosition = indexerMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case L4 -> indexerMotor.setControl(indexReq.withVelocity(1));
        }
    }



    public enum INDEXER_STATE {
        FEEDER,
        L2_CORAL,
        L3_CORAL,
        L4,
        L2_ALGAE,
        L3_ALGAE
    }

    public void setWantedState(INDEXER_STATE state) {
        wantedState = state;
    }
}
