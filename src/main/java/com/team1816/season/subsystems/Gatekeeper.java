package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Gatekeeper extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "gatekeeper";

    private GatekeeperState wantedState = GatekeeperState.CLOSED;

    //MOTORS
    private final IMotor topMotor = (IMotor)factory.getDevice(NAME, "topMotor");
    private final IMotor bottomMotor = (IMotor)factory.getDevice(NAME, "bottomMotor");

    private final VelocityVoltage voltageReq = new VelocityVoltage(0);

    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MIN_TOP_MOTOR_CLAMP = 0;
    private static final double MAX_TOP_MOTOR_CLAMP = 80;
    private static final double MIN_BOTTOM_MOTOR_CLAMP = 0;
    private static final double MAX_BOTTOM_MOTOR_CLAMP = 80;

    public Gatekeeper() {
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

    public void adjustCurrentTopMotorSetPoint(double adjustValue){
        wantedState.adjustTopMotorValue(adjustValue);

        GreenLogger.log(NAME+"/topMotor/"+wantedState.name()+"_adjusted_value/"+wantedState.getTopMotorValue());
    }

    public void adjustCurrentBottomMotorSetPoint(double adjustValue){
        wantedState.adjustBottomMotorValue(adjustValue);

        GreenLogger.log(NAME+"/bottomMotor/"+wantedState.name()+"_adjusted_value/"+wantedState.getBottomMotorValue());
    }

    private void applyState() {
        setTopVelocity(wantedState.getTopMotorValue());
        setBottomVelocity(wantedState.getBottomMotorValue());
    }

    private void setTopVelocity(double velocity) {
        double clampedVelocity = MathUtil.clamp(velocity, MIN_TOP_MOTOR_CLAMP, MAX_TOP_MOTOR_CLAMP);

        topMotor.setControl(voltageReq.withVelocity(clampedVelocity));
    }

    private void setBottomVelocity(double velocity) {
        double clampedVelocity = MathUtil.clamp(velocity, MIN_BOTTOM_MOTOR_CLAMP, MAX_BOTTOM_MOTOR_CLAMP);

        bottomMotor.setControl(voltageReq.withVelocity(clampedVelocity));
    }

    public void setWantedState(GatekeeperState state) {
        wantedState = state;
    }

    /**
     * Gets the current value of {@link #wantedState} for the gatekeeper.
     *
     * @return The current {@link #wantedState}.
     */
    public GatekeeperState getState() {
        return wantedState;
    }

    public enum GatekeeperState {
        OPEN(factory.getConstant(NAME, "topOpenVelocity", 0), factory.getConstant(NAME, "bottomOpenVelocity", 0)),
        CLOSED(factory.getConstant(NAME, "topClosedVelocity", 0), factory.getConstant(NAME, "bottomClosedVelocity", 0));

        private double topMotorValue, bottomMotorValue;

        GatekeeperState(double topMotorValue, double bottomMotorValue){
            this.topMotorValue = topMotorValue;
            this.bottomMotorValue = bottomMotorValue;
        }

        private void adjustTopMotorValue(double adjustValue) {
            this.topMotorValue = adjustValue;
        }

        private void adjustBottomMotorValue(double adjustValue) {
            this.bottomMotorValue += adjustValue;
        }

        public double getTopMotorValue() {
            return topMotorValue;
        }

        public double getBottomMotorValue() {
            return bottomMotorValue;
        }
    }
}
