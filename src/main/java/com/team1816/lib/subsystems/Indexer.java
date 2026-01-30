package com.team1816.lib.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

//Spin, Run, and Agitate
//Detects if Balls are in or not
//Agitate will unjam
//Spin will
public class Indexer extends SubsystemBase implements ITestableSubsystem {
    String NAME = "indexer";

    private final IMotor index = (IMotor) factory.getDevice(NAME, "indexMotor");
    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private INDEX_STATE wantedState = INDEX_STATE.INDEX_STOP;
    private Timer agitateTimer = new Timer();

    public enum INDEX_STATE{
        INDEX_STOP,
        INDEX_RUN,
        INDEX_AGITATE,
        INDEX_REVERSE
    }
    //java has an equivalent of a time, so when you set the index to one set it to current time, next time you come into loop
    public void periodic() {
        readFromHardware();
        applyState();
        agitateTimer.start();
    }

    public void setWantedState(Indexer.INDEX_STATE state) {
        this.wantedState = state;
    }

    private void applyState(){
        switch (wantedState) {
            case INDEX_RUN:
                setIndexSpeed(1);
                break;
            case INDEX_AGITATE:
                if(agitateTimer.hasElapsed(100)){
                    setIndexSpeed(1);
                }
                if(agitateTimer.hasElapsed(200)){
                    setIndexSpeed(-1);
                    agitateTimer.reset();
                }
                break;
            case INDEX_REVERSE:
                setIndexSpeed(-1);
                break;
            case INDEX_STOP:
                setIndexSpeed(0);
                break;
            default:
                break;
        }
    }

    public void setIndexSpeed(double wantedSpeed){
        SmartDashboard.putNumber("Index Velocity Voltage", wantedSpeed);

        index.setControl(velocityControl.withVelocity(wantedSpeed));
    }
}
