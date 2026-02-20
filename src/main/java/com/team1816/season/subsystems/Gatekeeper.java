package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Gatekeeper extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "gatekeeper";
    private final IMotor topMotor = (IMotor)factory.getDevice(NAME, "topMotor");
    private final IMotor bottomMotor = (IMotor)factory.getDevice(NAME, "bottomMotor");
    private GATEKEEPER_STATE wantedState = GATEKEEPER_STATE.CLOSED;
    private VelocityVoltage voltageReq = new VelocityVoltage(0);

    //YAML VALUES
    private double topClosedVelocity = factory.getConstant(NAME, "topClosedVelocity", 0);
    private double topOpenVelocity = factory.getConstant(NAME, "topOpenVelocity", 0);
    private double bottomClosedVelocity = factory.getConstant(NAME, "bottomClosedVelocity", 0);
    private double bottomOpenVelocity = factory.getConstant(NAME, "bottomOpenVelocity", 0);

    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MIN_TOP_MOTOR_CLAMP = 0;
    private static final double MAX_TOP_MOTOR_CLAMP = 80;
    private static final double MIN_BOTTOM_MOTOR_CLAMP = 0;
    private static final double MAX_BOTTOM_MOTOR_CLAMP = 80;

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
            case OPEN:
                // TODO: Add correct velocity
                setTopVelocity(topOpenVelocity);
                setBottomVelocity(bottomOpenVelocity);
                break;
            case CLOSED:
                setTopVelocity(topClosedVelocity);
                setBottomVelocity(bottomClosedVelocity);
                break;
            default:
                break;
        }

        SmartDashboard.putString("Gatekeeper state: ", wantedState.toString());
    }

    public enum GATEKEEPER_STATE {
        OPEN,
        CLOSED
    }

    private void setTopVelocity(double velocity) {
        double clampedVelocity = MathUtil.clamp(velocity, MIN_TOP_MOTOR_CLAMP, MAX_TOP_MOTOR_CLAMP);

        topMotor.setControl(voltageReq.withVelocity(clampedVelocity));
    }

    private void setBottomVelocity(double velocity) {
        double clampedVelocity = MathUtil.clamp(velocity, MIN_BOTTOM_MOTOR_CLAMP, MAX_BOTTOM_MOTOR_CLAMP);

        bottomMotor.setControl(voltageReq.withVelocity(clampedVelocity));
    }

    public void setWantedState(GATEKEEPER_STATE state) {
        wantedState = state;
    }
}
