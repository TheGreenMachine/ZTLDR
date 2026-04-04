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

    public Feeder() {
        super();

        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
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
        FEEDING(factory.getConstant(NAME, "feedingDutyCycle", 0),
            factory.getConstant(NAME, "agitateForwardDutyCycle", .15)),
        STOPPED(factory.getConstant(NAME, "stoppedDutyCycle", 0),
            factory.getConstant(NAME, "agitateStoppedDutyCycle", 0));

        private final double feedMotorDutyCycle;
        private final double agitateMotorDutyCycle;

        FeederState(double feedMotorDutyCycle, double agitateMotorDutyCycle) {
            this.feedMotorDutyCycle = feedMotorDutyCycle;
            this.agitateMotorDutyCycle = agitateMotorDutyCycle;
        }

        public double getFeedMotorDutyCycle() {
            return feedMotorDutyCycle;
        }

        public double getAgitateMotorDutyCycle() {
            return agitateMotorDutyCycle;
        }
    }
}
