package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Gatekeeper extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "gatekeeper";
    private final IMotor gatekeeperTopMotor = (IMotor)factory.getDevice(NAME, "gatekeeperTopMotor");
    private final IMotor gatekeeperBottomMotor = (IMotor)factory.getDevice(NAME, "gatekeeperBottomMotor");
    private double curTopPosition;
    private double curBottomPosition;
    private GATEKEEPER_STATE wantedState = GATEKEEPER_STATE.CLOSED;
    VelocityVoltage indexReq = new VelocityVoltage(0);
    private VoltageOut voltageControl = new VoltageOut(0);

    @Override
    public void periodic() {
        readFromHardware();
        applyState();

        CommandScheduler.getInstance().getDefaultButtonLoop().poll();
    }

    @Override
    public void readFromHardware() {
        curTopPosition = gatekeeperTopMotor.getMotorPosition();
        curBottomPosition = gatekeeperBottomMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case OPEN:
                // TODO: Add correct velocity
                setGatekeeperTopVelocity(10);
                setGatekeeperBottomVelocity(5);
                break;
            case CLOSED:
            default:
                setGatekeeperTopVelocity(0);
                setGatekeeperBottomVelocity(0);
                break;
        }

        SmartDashboard.putString("Gatekeeper state: ", wantedState.toString());
    }



    public enum GATEKEEPER_STATE {
        OPEN,
        CLOSED
    }

    private void setGatekeeperTopVelocity(double wantedVelocity) {
        gatekeeperTopMotor.setControl(voltageControl.withOutput(wantedVelocity));
    }

    private void setGatekeeperBottomVelocity(double wantedVelocity) {
        gatekeeperBottomMotor.setControl(voltageControl.withOutput(wantedVelocity));
    }

    public void setWantedState(GATEKEEPER_STATE state) {
        wantedState = state;
    }
}
