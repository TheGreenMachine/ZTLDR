package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Indexer extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "indexer";
    private final IMotor indexerMotor = (IMotor) factory.getDevice(NAME, "indexerMotor");
    VelocityVoltage indexReq = new VelocityVoltage(0);
    //    private final IMotor indexerMotorTop = (IMotor)factory.getDevice(NAME, "indexerMotorTop"); //May need to implement
    private double curPosition;
    private INDEXER_STATE wantedState = INDEXER_STATE.IDLING;

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
            case PASSIVE_FEEDING:

                break;
            case ACTIVE_FEEDING:

                break;
            case AGITATING:

                break;
            case IDLING:

                break;
            default:
                break;
        }

        SmartDashboard.putString("Indexer state: ", wantedState.toString());
    }

    public void setWantedState(INDEXER_STATE state) {
        wantedState = state;
    }

    public enum INDEXER_STATE {
        PASSIVE_FEEDING,
        ACTIVE_FEEDING,
        AGITATING,
        IDLING
    }
}
