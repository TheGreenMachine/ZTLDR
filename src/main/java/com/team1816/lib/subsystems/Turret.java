package com.team1816.lib.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Turret extends SubsystemBase implements ITestableSubsystem {
    String NAME = "turret";

    private double GEAR_RATIO = 1.0;

    private double wantedAngle = 0.0;

    private final IMotor turret = (IMotor) factory.getDevice(NAME, "turretMotor");
    private PositionVoltage positionControl = new PositionVoltage(0);
    private TURRET_STATE wantedState = TURRET_STATE.TURRET_IDLE;

    public double currentVoltage = 0;
    public double currentPosition = 0;

    public enum TURRET_STATE {
        TURRET_TO_0,
        TURRET_TO_180,
        TURRET_TO_ANGLE,
        TURRET_IDLE
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(TURRET_STATE state) {
        this.wantedState = state;
    }

    public void setWantedState(TURRET_STATE state, double angle) {
        this.wantedState = state;
        this.wantedAngle = angle;
    }

    @Override
    public void readFromHardware() {
        currentPosition = turret.getMotorPosition();
        currentVoltage = 0;
    }

    private void applyState() {
        switch (wantedState) {
            case TURRET_TO_0:
                setTurretAngle(0);
                break;
            case TURRET_TO_180:
                setTurretAngle(180);
                break;
            case TURRET_TO_ANGLE:
                setTurretAngle(wantedAngle);
                break;
            case TURRET_IDLE:
            default:
                break;
        }
    }

    public void setTurretAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        turret.setControl(positionControl.withPosition(rotations));
    }

}
