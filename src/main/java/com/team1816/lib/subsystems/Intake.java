package com.team1816.lib.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.text.Position;

import java.time.Duration;
import java.time.Instant;

import static com.team1816.lib.Singleton.factory;

public class Intake extends SubsystemBase implements ITestableSubsystem {
    private static final double GEAR_RATIO = 1;

    String NAME = "intake";

    private final IMotor intake = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipper = (IMotor) factory.getDevice(NAME, "flipperMotor");
    //private VoltageOut voltageControl = new VoltageOut(0);
    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private INTAKE_WANTED_STATE wantedState = INTAKE_WANTED_STATE.INTAKE_IDLE;
    private INTAKE_SYSTEM_STATE systemState = INTAKE_SYSTEM_STATE.INTAKE_IDLE;

    public double currentVoltage = 0;
    public double currentPosition = 0;

    public double currentFlipperAngle = 67;
    Instant descentStart;

    public enum INTAKE_WANTED_STATE {
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_IDLE

    }

    public enum INTAKE_SYSTEM_STATE {
        INTAKE_IN,
        INTAKE_TRANSITIONING,
        INTAKE_OUT,
        INTAKE_IDLE,
    }
    public void periodic() {
        readFromHardware();
        systemState = progressTransition();
        applyState();
    }

    public void setWantedState(INTAKE_WANTED_STATE state) {
        if(systemState == INTAKE_SYSTEM_STATE.INTAKE_IDLE) {
            descentStart = Instant.now();
        }

        this.wantedState = state;
    }

    @Override
    public void readFromHardware() {
        currentPosition = intake.getMotorPosition();
        currentFlipperAngle = (flipper.getMotorPosition() / GEAR_RATIO) * 360; // maybe
        currentVoltage = 0;
    }

    private INTAKE_SYSTEM_STATE progressTransition() {
        switch (wantedState) {
            case INTAKE_OUT:
            case INTAKE_IN:
                final double threshold = 6;
                final double targetAngle = 225;

                final long timeOverride = 4;
                final Instant currentTime = Instant.now();

                final INTAKE_SYSTEM_STATE out = wantedState == INTAKE_WANTED_STATE.INTAKE_IN ?
                    INTAKE_SYSTEM_STATE.INTAKE_IN : INTAKE_SYSTEM_STATE.INTAKE_OUT;

                if((currentFlipperAngle > targetAngle - threshold && currentFlipperAngle < targetAngle + threshold)
                    || Duration.between(currentTime, descentStart).toSeconds() <= timeOverride) {
                    return out;
                }

                return INTAKE_SYSTEM_STATE.INTAKE_TRANSITIONING;
            case INTAKE_IDLE:
            default:
                return INTAKE_SYSTEM_STATE.INTAKE_IDLE;
        }
    }


    private void applyState() {
        switch (systemState) {
            case INTAKE_IN:
            case INTAKE_OUT:
                final double speed = 10 * (systemState == INTAKE_SYSTEM_STATE.INTAKE_IN ? 1 : -1);
                setTurretSpeed(speed);
            case INTAKE_TRANSITIONING:
                setFlipperAngle(225);
            break;

            case INTAKE_IDLE:
            default:
                setTurretSpeed(0);
                setFlipperAngle(45);
                break;
        }
    }

    public void setFlipperAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        flipper.setControl(positionControl.withPosition(rotations));
    }

    public void setTurretSpeed(double wantedSpeed) {
        SmartDashboard.putNumber("Intake Velocity Voltage", wantedSpeed);

        intake.setControl(velocityControl.withVelocity(wantedSpeed));
    }
}
