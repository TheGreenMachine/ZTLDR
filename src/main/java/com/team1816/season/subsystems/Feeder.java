package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Feeder extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "feeder";
    private final IMotor feederMotor = (IMotor)factory.getDevice(NAME, "feederMotor");
//    private final IMotor feederMotorTop = (IMotor)factory.getDevice(NAME, "feederMotorTop"); //May need to implement
    private double curPosition;
    private FEEDER_STATE wantedState = FEEDER_STATE.IDLING;
    VelocityVoltage indexReq = new VelocityVoltage(0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();

        CommandScheduler.getInstance().getDefaultButtonLoop().poll();
    }

    @Override
    public void readFromHardware() {
        curPosition = feederMotor.getMotorPosition();
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

        SmartDashboard.putString("Feeder state: ", wantedState.toString());
    }



    public enum FEEDER_STATE {
        PASSIVE_FEEDING,
        ACTIVE_FEEDING,
        AGITATING,
        IDLING
    }

    public void setWantedState(FEEDER_STATE state) {
        wantedState = state;
    }
}
