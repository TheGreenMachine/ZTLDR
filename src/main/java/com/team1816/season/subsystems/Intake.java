package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

    private boolean hasCycleBeenSetForTheFirstTime = false;

    private double previousDutyCycle = 20;

    private FlipperPosition previousPosition = FlipperPosition.OUT;

    //MOTORS
    private final IMotor intakeMotor = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipperMotor = (IMotor) factory.getDevice(NAME, "flipperMotor");

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final TorqueCurrentFOC flipperMotorTorqueCurrentRequest = new TorqueCurrentFOC(0);

    /**
     * The minimum position of the flipper motor to count it as far enough out to switch to our
     * lower current to just hold it out instead of pushing it out.
     */
    private final double FLIPPER_MOTOR_OUT_POSITION;
    private final double FLIPPER_MOTOR_PULL_IN_ONE_POSITION;
    private final double FLIPPER_MOTOR_PULL_IN_TWO_POSITION;
    private final double FLIPPER_MOTOR_IN_POSITION;
    private final double FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES;
    private final double FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES;
    private final double FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES;
    private final double FLIPPER_MOTOR_HOLD_IN_CURRENT_AMPERES;

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
        FLIPPER_MOTOR_PULL_IN_ONE_POSITION = factory.getConstant(NAME, "flipperMotorPullInOnePosition", 0);
        FLIPPER_MOTOR_PULL_IN_TWO_POSITION = factory.getConstant(NAME, "flipperMotorPullInTwoPosition", 0);
        FLIPPER_MOTOR_IN_POSITION = factory.getConstant(NAME, "flipperMotorInPosition", 0);
        FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorRetractCurrentAmperes", 0);
        FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorExtendCurrentAmperes", 0);
        FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorHoldOutCurrentAmperes", 0);
        FLIPPER_MOTOR_HOLD_IN_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorHoldInCurrentAmperes", 0);

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

        if (!hasCycleBeenSetForTheFirstTime || previousDutyCycle != wantedIntakeDutyCycle && previousPosition != wantedFlipperPosition) {

            setIntakeSpeed(wantedIntakeDutyCycle);
            setFlipperPosition(wantedFlipperPosition);

            previousPosition = wantedFlipperPosition;
            previousDutyCycle = wantedIntakeDutyCycle;
            hasCycleBeenSetForTheFirstTime = true;
        }
    }

    private void setIntakeSpeed(double velocity) {
        intakeMotor.setControl(dutyCycleOut.withOutput(velocity));
    }

    private void setFlipperPosition(FlipperPosition position) {
        switch (position) {
            case IN -> {
                if (flipperMotor.getMotorPosition() > FLIPPER_MOTOR_IN_POSITION) {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES)
                    );
                }
                else {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_HOLD_IN_CURRENT_AMPERES)
                    );
                }
            }
            case OUT -> {
                if (flipperMotor.getMotorPosition() < FLIPPER_MOTOR_OUT_POSITION) {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES)
                    );
                }
                else {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES)
                    );
                }
            }
            case PULL_IN_ONE -> {
                if (flipperMotor.getMotorPosition() > FLIPPER_MOTOR_PULL_IN_ONE_POSITION) {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES)
                    );
                }
                else {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(0)
                    );
                }
            }
            case PULL_IN_TWO -> {
                if (flipperMotor.getMotorPosition() > FLIPPER_MOTOR_PULL_IN_TWO_POSITION) {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES)
                    );
                }
                else {
                    flipperMotor.setControl(
                        flipperMotorTorqueCurrentRequest.withOutput(0)
                    );
                }
            }
        }
    }

    public void setWantedState(IntakeState state) {
        this.wantedState = state;
    }

    /**
     * The position of the flipper. This is not just a number, since we are doing a custom current
     * based control, so the behavior is completely different for going in and out.
     */
    public enum FlipperPosition {
        IN,
        OUT,
        PULL_IN_ONE,
        PULL_IN_TWO
    }

    public enum IntakeState {
        STOW(
            factory.getConstant(NAME, "intakeOffSpeed", 0, true),
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
        ),
        PULL_IN_ONE(
            factory.getConstant(NAME, "intakeOnSpeed", 0),
            FlipperPosition.PULL_IN_ONE
        ),
        PULL_IN_TWO(
            factory.getConstant(NAME, "intakeOnSpeed", 0),
            FlipperPosition.PULL_IN_TWO
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

        private FlipperPosition getFlipperMotorPosition() {
            return flipperMotorPosition;
        }
    }
}
