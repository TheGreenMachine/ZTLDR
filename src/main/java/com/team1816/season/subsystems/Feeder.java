package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Feeder extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "feeder";

    private FeederState wantedState = FeederState.STOPPED;

    //MOTORS
    private final IMotor feedMotor = (IMotor)factory.getDevice(NAME, "feedMotor");

    private final DutyCycleOut feedMotorDutyCycle = new DutyCycleOut(0);

    public Feeder() {
        super();
        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
    }

    public void adjustCurrentFeedMotorSetPoint(double adjustValue){
        wantedState.adjustFeedMotorValue(adjustValue);

        GreenLogger.log(NAME+"/feedMotor/"+wantedState.name()+"_adjusted_value/"+wantedState.getFeedMotorDutyCycle());
    }

    private void applyState() {
        feedMotor.setControl(feedMotorDutyCycle.withOutput(wantedState.getFeedMotorDutyCycle()));
    }

    public void setWantedState(FeederState state) {
        wantedState = state;
    }

    public enum FeederState {
        FEEDING(factory.getConstant(NAME, "feedingDutyCycle", 0)),
        STOPPED(factory.getConstant(NAME, "stoppedDutyCycle", 0));

        private double feedMotorDutyCycle;

        FeederState(double feedMotorDutyCycle){
            this.feedMotorDutyCycle = feedMotorDutyCycle;
        }

        private void adjustFeedMotorValue(double adjustValue) {
            this.feedMotorDutyCycle += adjustValue;
        }

        public double getFeedMotorDutyCycle() {
            return feedMotorDutyCycle;
        }
    }
}
