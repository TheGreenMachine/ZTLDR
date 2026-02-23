package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Feeder extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "feeder";

    private FEEDER_STATE wantedState = FEEDER_STATE.IDLING;

    //MOTORS
    private final IMotor feedMotor = (IMotor)factory.getDevice(NAME, "feedMotor");

    private final VelocityVoltage velocityReq = new VelocityVoltage(0);

    //YAML VALUES
    private double fastFeedingVelocity = Singleton.factory.getConstant(NAME, "fastFeeding", 0);
    private double slowFeedingVelocity = Singleton.factory.getConstant(NAME, "slowFeeding", 0);
    private double reversingVelocity = Singleton.factory.getConstant(NAME, "reversing", 0);
    private double idlingVelocity = Singleton.factory.getConstant(NAME, "idling", 0);

    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MIN_FEED_MOTOR_CLAMP = -80;
    private static final double MAX_FEED_MOTOR_CLAMP = 80;

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
    }

    private void applyState() {
        switch (wantedState) {
            case SLOW_FEEDING:
                setFeedVelocity(slowFeedingVelocity);
                break;
            case FAST_FEEDING:
                setFeedVelocity(fastFeedingVelocity);
                break;
            case REVERSING:
                setFeedVelocity(reversingVelocity);
                break;
            case IDLING:
                setFeedVelocity(idlingVelocity);
                break;
            default:
                break;
        }

        GreenLogger.log("Feeder state: " + wantedState.toString());
    }

    private void setFeedVelocity(double feedVelocity){
        double clampedVelocity = MathUtil.clamp(feedVelocity, MIN_FEED_MOTOR_CLAMP, MAX_FEED_MOTOR_CLAMP);

        feedMotor.setControl(velocityReq.withVelocity(clampedVelocity));
    }

    public enum FEEDER_STATE {
        SLOW_FEEDING,
        FAST_FEEDING,
        REVERSING,
        IDLING
    }

    public void setWantedState(FEEDER_STATE state) {
        wantedState = state;
    }
}
