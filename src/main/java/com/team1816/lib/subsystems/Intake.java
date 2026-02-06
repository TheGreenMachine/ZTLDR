package com.team1816.lib.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.Duration;
import java.time.Instant;

import static com.team1816.lib.Singleton.factory;

public class Intake extends SubsystemBase implements ITestableSubsystem {
    static final String NAME = "intake";

    private static final double GEAR_RATIO = 1;

    private final IMotor intake = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipper = (IMotor) factory.getDevice(NAME, "flipperMotor");

    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private INTAKE_STATE wantedState = INTAKE_STATE.INTAKE_UP;

    public double currentVoltage = 0;
    public double currentPosition = 0;

    public double currentFlipperAngle = 67;
    private Instant descentStart = Instant.now();

    public enum INTAKE_STATE {
        INTAKE_IN(
            factory.getConstant(NAME, "inSpeed", 10, true),
            factory.getConstant(NAME, "inAngle", 10, true)
        ),
        INTAKE_OUT(
            factory.getConstant(NAME, "outSpeed", -10, true),
            factory.getConstant(NAME, "outAngle", 225, true)
        ),
        INTAKE_DOWN(
            0,
            factory.getConstant(NAME, "downAngle", 225, true)
        ),
        INTAKE_UP(
            0,
            factory.getConstant(NAME, "upAngle", 45, true)
        );

        private final double speed, angle;
        INTAKE_STATE (double speed, double angle) {
            this.speed = speed;
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
        public double getSpeed() {
            return speed;
        }
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(INTAKE_STATE state) {
        if((state == INTAKE_STATE.INTAKE_IN || state == INTAKE_STATE.INTAKE_OUT) && wantedState == INTAKE_STATE.INTAKE_UP) {
            descentStart = Instant.now();
        }

        this.wantedState = state;
    }

    @Override
    public void readFromHardware() {
        currentPosition = intake.getMotorPosition();
        currentFlipperAngle = (flipper.getMotorPosition() / GEAR_RATIO) * 360;
        currentVoltage = 0;
    }

    private boolean canSuckOrBlow() {
        final double targetAngle = 225;
        final double threshold = 6;

        final long timeOverride = 4;
        final Instant currentTime = Instant.now();

        return ((currentFlipperAngle > targetAngle - threshold && currentFlipperAngle < targetAngle + threshold)
            || Duration.between(currentTime, descentStart).toSeconds() <= timeOverride) ;
    }

    private void applyState() {
        double intakeSpeed = canSuckOrBlow() ? wantedState.getSpeed() : 0;
        double flipperAngle = wantedState.getAngle();

        setTurretSpeed(intakeSpeed);
        setFlipperAngle(flipperAngle);
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
