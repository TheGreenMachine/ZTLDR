package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Intake extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "intake";

    private IntakeState wantedState = IntakeState.STOW;

    private FlipperPosition wantedFlipperPosition = FlipperPosition.IN;
    private double wantedIntakeDutyCycle = 0;

    //MOTORS
    private final IMotor intakeMotor = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipperMotor = (IMotor) factory.getDevice(NAME, "flipperMotor");

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicExpoVoltage flipperMotorPositionRequest = new MotionMagicExpoVoltage(0);

    private final double FLIPPER_MOTOR_OUT_POSITION;
    private final double FLIPPER_MOTOR_IN_POSITION;

    //MECHANISMS
    private double currentPosition = 0;
    private static final double ROTATIONS_TO_DEGREES_RATIO = 1;
    private Mechanism2d intakeMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    private MechanismRoot2d intakeMechRoot = intakeMech.getRoot("Intake Root", 0, 1);
    private MechanismLigament2d intakeAngleML = intakeMechRoot.append(
        new MechanismLigament2d("Intake Angle", 1.5, 0));

    public Intake () {
        super();
        SmartDashboard.putData("Intake", intakeMech);
        FLIPPER_MOTOR_OUT_POSITION = factory.getConstant(NAME, "flipperMotorOutPosition", 0);
        FLIPPER_MOTOR_IN_POSITION = factory.getConstant(NAME, "flipperMotorInPosition", 0);

        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
        GreenLogger.periodicLog(NAME + "/Wanted Flipper Position", () -> wantedFlipperPosition);
        GreenLogger.periodicLog(NAME + "/Wanted Intake Duty Cycle", () -> wantedIntakeDutyCycle);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        currentPosition = flipperMotor.getMotorPosition();

        intakeAngleML.setAngle(currentPosition * ROTATIONS_TO_DEGREES_RATIO);
    }

    private void applyState() {
        wantedIntakeDutyCycle = wantedState.getIntakeMotorValue();
        wantedFlipperPosition = wantedState.getFlipperMotorPosition();

        setIntakeSpeed(wantedIntakeDutyCycle);
        setFlipperPosition(wantedFlipperPosition);
    }

    private void setIntakeSpeed(double velocity) {
        intakeMotor.setControl(dutyCycleOut.withOutput(velocity));
    }

    public IntakeState getState() {
        return wantedState;
    }

    private void setFlipperPosition(FlipperPosition position) {
        switch (position) {
            case IN -> flipperMotor.setControl(flipperMotorPositionRequest.withPosition(FLIPPER_MOTOR_IN_POSITION));
            case OUT -> flipperMotor.setControl(flipperMotorPositionRequest.withPosition(FLIPPER_MOTOR_OUT_POSITION));
        }
    }

    public void setWantedState(IntakeState state) {
        this.wantedState = state;
    }

    public enum FlipperPosition {
        IN,
        OUT
    }

    public enum IntakeState {
        STOW(
            factory.getConstant(NAME, "intakeSlowSpeed", .1, true),
            FlipperPosition.IN
        ),
        INTAKE(
            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
            FlipperPosition.OUT
        ),
        OUTTAKE(
            factory.getConstant(NAME, "outtakeSpeed", -0.5, true),
            FlipperPosition.OUT
        ),
        STOP_OUT(
            0,
            FlipperPosition.OUT
        );

        private double intakeMotorValue;
        private final FlipperPosition flipperMotorPosition;

        IntakeState(double speed, FlipperPosition flipperMotorPosition) {
            this.intakeMotorValue = speed;
            this.flipperMotorPosition = flipperMotorPosition;
        }

        private void adjustIntakeMotorSetPoint(double adjustValue){
            this.intakeMotorValue += adjustValue;
        }

        public double getIntakeMotorValue() {
            return intakeMotorValue;
        }

        public FlipperPosition getFlipperMotorPosition() {
            return flipperMotorPosition;
        }
    }
}
