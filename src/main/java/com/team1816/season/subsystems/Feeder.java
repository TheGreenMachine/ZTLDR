package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Feeder extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "feeder";
    private final IMotor feedMotor = (IMotor)factory.getDevice(NAME, "feedMotor");
    private double currentVelocity;
    private FEEDER_STATE wantedState = FEEDER_STATE.IDLING;
    private VelocityVoltage velocityReq = new VelocityVoltage(0);

    //VELOCITY AND POSITION VALUES
    public final double FEED_VELOCITY_FAST_FEEDING = Singleton.factory.getConstant(NAME, "fastFeeding", 0);
    public final double FEED_VELOCITY_SLOW_FEEDING = Singleton.factory.getConstant(NAME, "slowFeeding", 0);
    public final double FEED_VELOCITY_REVERSING = Singleton.factory.getConstant(NAME, "reversing", 0);
    public final double FEED_VELOCITY_IDLING = Singleton.factory.getConstant(NAME, "idling", 0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        currentVelocity = feedMotor.getMotorVelocity();
    }

    private void applyState() {
        switch (wantedState) {
            case SLOW_FEEDING:
                setFeedVelocity(FEED_VELOCITY_SLOW_FEEDING);
                break;
            case FAST_FEEDING:
                setFeedVelocity(FEED_VELOCITY_FAST_FEEDING);
                break;
            case REVERSING:
                setFeedVelocity(FEED_VELOCITY_REVERSING);
                break;
            case IDLING:
                setFeedVelocity(FEED_VELOCITY_IDLING);
                break;
            default:
                break;
        }

        SmartDashboard.putString("Feeder state: ", wantedState.toString());
    }

    private void setFeedVelocity(double feedVelocity){
        double clampedVelocity = MathUtil.clamp(feedVelocity, -80, 80);

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
