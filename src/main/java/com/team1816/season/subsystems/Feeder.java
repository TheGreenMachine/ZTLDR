package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Timer;
import java.util.TimerTask;

import static com.team1816.lib.Singleton.factory;

public class Feeder extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "feeder";

    private FeederState wantedState = FeederState.STOPPED;

    //MOTORS
    private final IMotor feedMotor = (IMotor)factory.getDevice(NAME, "feedMotor");
    private final IMotor agitateMotor = (IMotor)factory.getDevice(NAME, "agitateMotor");

    private final DutyCycleOut feedMotorDutyCycle = new DutyCycleOut(0);
    private final DutyCycleOut agitateMotorDutyCycle = new DutyCycleOut(0);

    private final double agitateForwardDutyCycle = factory.getConstant(NAME, "agitateForwardDutyCycle", 0.05);
    private final double agitateReverseDutyCycle = factory.getConstant(NAME, "agitateReverseDutyCycle", -0.05);
    private final double agitateCycleTime = factory.getConstant(NAME, "agitateCycleTime", 3000);

    private final Timer agitateTimer = new Timer();
    private boolean agitateLeft = false;
    private final TimerTask agitateTimerTask = new TimerTask(){
        @Override
        public void run() {
            if (agitateLeft) {
                agitateLeft = false;
                agitateMotor.setControl(agitateMotorDutyCycle.withOutput(agitateForwardDutyCycle));
            } else {
                agitateLeft = true;
                agitateMotor.setControl(agitateMotorDutyCycle.withOutput(agitateReverseDutyCycle));
            }
        }
    };

    public Feeder() {
        super();
        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
        GreenLogger.periodicLog(NAME + "/Agitator Timer", () -> agitateCycleTime);
        GreenLogger.periodicLog(NAME + "/Agitate Left Speed", () -> agitateForwardDutyCycle);
        GreenLogger.periodicLog(NAME + "/Agitate Right Speed", () -> agitateReverseDutyCycle);
        GreenLogger.periodicLog(NAME + "/Agitate Left Flag", () -> agitateLeft);

        agitateTimer.schedule(agitateTimerTask, 0, (long)agitateCycleTime);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    private void applyState() {
        feedMotor.setControl(feedMotorDutyCycle.withOutput(wantedState.getFeedMotorDutyCycle()));
        agitateMotor.setControl(agitateMotorDutyCycle.withOutput(wantedState.getAgitateMotorDutyCycle()));

    }

    public void setWantedState(FeederState state) {
        wantedState = state;
    }



    public enum FeederState {
        FEEDING(factory.getConstant(NAME, "feedingDutyCycle", 0)),
        STOPPED(factory.getConstant(NAME, "stoppedDutyCycle", 0)),
        REVERSING(factory.getConstant(NAME, "reversingDutyCycle", 0));

        private double feedMotorDutyCycle;
        private double agitateMotorDutyCycle;

        FeederState(double feedMotorDutyCycle){
            this.feedMotorDutyCycle = feedMotorDutyCycle;
        }

        private void adjustFeedMotorValue(double adjustValue) {
            this.feedMotorDutyCycle += adjustValue;
        }

        public double getFeedMotorDutyCycle() {
            return feedMotorDutyCycle;
        }
        public double getAgitateMotorDutyCycle() {
            return agitateMotorDutyCycle;
        }
    }
}
