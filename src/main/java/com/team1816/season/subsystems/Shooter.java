package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {
    String NAME = "shooter";

    private double GEAR_RATIO = 1.0;

    private double wantedAngle = 0.0;

    private final IMotor shooterMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");
    private VoltageOut voltageControl = new VoltageOut(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private SHOOTER_STATE wantedState = SHOOTER_STATE.SHOOTER_IDLE;

    public double currentVoltage = 0;
    public double currentPosition = 0;

    public enum SHOOTER_STATE {
        SHOOTER_TO_0,
        SHOOTER_TO_180,
        SHOOTER_TO_ANGLE,
        SHOOTER_ROTATE_LEFT,
        SHOOTER_ROTATE_RIGHT,
        SHOOTER_IDLE
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setWantedState(SHOOTER_STATE state, double angle) {
        this.wantedState = state;
        this.wantedAngle = angle;
    }

    @Override
    public void readFromHardware() {
        currentPosition = shooterMotor.getMotorPosition();
        currentVoltage = 0;
    }

    private void applyState() {
        switch (wantedState) {
            case SHOOTER_TO_0:
                setShooterAngle(0);
                break;
            case SHOOTER_TO_180:
                setShooterAngle(180);
                break;
            case SHOOTER_TO_ANGLE:
                setShooterAngle(wantedAngle);
                break;
            case SHOOTER_ROTATE_LEFT:
                setShooterSpeed(1);
                break;
            case SHOOTER_ROTATE_RIGHT:
                setShooterSpeed(-1);
                break;
            case SHOOTER_IDLE:
            default:
                setShooterSpeed(0);
                break;
        }
    }

    public void setShooterSpeed(double wantedSpeed) {
        double output = MathUtil.clamp(wantedSpeed, -12.0, 12.0);

        SmartDashboard.putNumber("Shooter Voltage", output);

        shooterMotor.setControl(voltageControl.withOutput(output));
    }

    public void setShooterAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        shooterMotor.setControl(positionControl.withPosition(rotations));
    }

}
