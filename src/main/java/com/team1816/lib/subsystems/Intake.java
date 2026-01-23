package com.team1816.lib.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.text.Position;

import static com.team1816.lib.Singleton.factory;

public class Intake extends SubsystemBase implements ITestableSubsystem {
    private static final double GEAR_RATIO = 1;
    String NAME = "intake";

    private final IMotor intake = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipper = (IMotor) factory.getDevice(NAME, "flipperMotor");
    //private VoltageOut voltageControl = new VoltageOut(0);
    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private INTAKE_STATE wantedState = INTAKE_STATE.INTAKE_IDLE;

    public double currentVoltage = 0;
    public double currentPosition = 0;

    public enum INTAKE_STATE {
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_IDLE
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(INTAKE_STATE state) {
        this.wantedState = state;
    }

    @Override
    public void readFromHardware() {
        currentPosition = intake.getMotorPosition();
        currentVoltage = 0;
    }

    private void applyState() {
        switch (wantedState) {
            case INTAKE_IN:
                setTurretSpeed(10);
                setFlipperAngle(225);
                break;
            case INTAKE_OUT:
                setTurretSpeed(-10);
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
