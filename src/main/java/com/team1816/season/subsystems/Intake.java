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

    private INTAKE_STATE wantedState = INTAKE_STATE.INTAKE_IN_AND_OFF;
    private INTAKE_STATE previousWantedState = INTAKE_STATE.INTAKE_IN_AND_OFF;

    //MOTORS
    private final IMotor intakeMotor = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipperMotor = (IMotor) factory.getDevice(NAME, "flipperMotor");

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final TorqueCurrentFOC flipperMotorTorqueCurrentRequest = new TorqueCurrentFOC(0);


    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MIN_INTAKE_MOTOR_CLAMP = -80;
    private static final double MAX_INTAKE_MOTOR_CLAMP = 80;
    private static final double MIN_FLIPPER_MOTOR_CLAMP = -0.11;
    private static final double MAX_FLIPPER_MOTOR_CLAMP = -0.033;
    /**
     * The minimum position of the flipper motor to count it as far enough out to switch to our
     * lower current to just hold it out instead of pushing it out.
     */
    private final double FLIPPER_MOTOR_OUT_MINIMUM_POSITION;
    private final double FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES;
    private final double FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES;
    private final double FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES;

    //MECHANISMS
    private double currentPosition = 0;
    private static final double ROTATIONS_TO_DEGREES_RATIO = 1;
    private Mechanism2d intakeMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    private MechanismRoot2d intakeMechRoot = intakeMech.getRoot("Intake Root", 0, 1);
    private MechanismLigament2d intakeAngleML = intakeMechRoot.append(
        new MechanismLigament2d("Intake Angle", 1.5, 0));

    FlipperPosition flipperPosition;

    public Intake () {
        super();
        //SmartDashboard.putData("Intake", intakeMech);
        FLIPPER_MOTOR_OUT_MINIMUM_POSITION = factory.getConstant(NAME, "flipperMotorOutMinimumPosition", 0);
        FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorRetractCurrentAmperes", 0);
        FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorExtendCurrentAmperes", 0);
        FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES = factory.getConstant(NAME, "flipperMotorHoldOutCurrentAmperes", 0);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void adjustCurrentIntakeMotorSetPoint(double adjustValue){
        wantedState.adjustIntakeMotorSetPoint(adjustValue);

        GreenLogger.log(NAME+"/intakeMotor/"+wantedState.name()+"_adjusted_value/"+wantedState.getIntakeMotorValue());
    }

    @Override
    public void readFromHardware() {
        currentPosition = flipperMotor.getMotorPosition();

        intakeAngleML.setAngle(currentPosition * ROTATIONS_TO_DEGREES_RATIO);
    }

    private void applyState() {
        double intakeSpeed = wantedState.getIntakeMotorValue();
        flipperPosition = wantedState.getFlipperMotorPosition();

        setIntakeSpeed(intakeSpeed);
        setFlipperPosition(flipperPosition);

        if (wantedState != previousWantedState) {
            GreenLogger.log("Intake state: " + wantedState.toString());
            GreenLogger.log("Intake speed: " + intakeSpeed);
            GreenLogger.log("Intake position: " + flipperPosition);

            //SmartDashboard.putString("Intake state: ", wantedState.toString());
            //SmartDashboard.putNumber("Intake speed: ", intakeSpeed);
            //SmartDashboard.putString("Intake position: ", String.valueOf(flipperPosition));
            previousWantedState = wantedState;
        }
    }

    private void setIntakeSpeed(double velocity) {
        intakeMotor.setControl(dutyCycleOut.withOutput(velocity));
    }

    private void setFlipperPosition(FlipperPosition position) {
        if (position == FlipperPosition.IN) {
            flipperMotor.setControl(
                flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES)
            );
        }
        else if (position == FlipperPosition.POSITION_1) {
            if (flipperMotor.getMotorPosition() > 0.105) {
                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES));
            }
            else {
                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(
                    FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES)
                );
            }
        }
        else if (position == FlipperPosition.POSITION_2) {
            if (flipperMotor.getMotorPosition() > 0.046) {
                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES));
            }
            else {
                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(
                    FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES)
                );
            }
        }
//        else if (position == FlipperPosition.POSITION_3) {
//            if (flipperMotor.getMotorPosition() > ) {
//                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES));
//            }
//        }
//        else if (position == FlipperPosition.POSITION_4) {
//            if (flipperMotor.getMotorPosition() > ) {
//                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_RETRACT_CURRENT_AMPERES));
//            }
//        }
        else {
            if (flipperMotor.getMotorPosition() < FLIPPER_MOTOR_OUT_MINIMUM_POSITION) {
                flipperMotor.setControl(
                    flipperMotorTorqueCurrentRequest.withOutput(FLIPPER_MOTOR_EXTEND_CURRENT_AMPERES)
                );
            }
            else {
                flipperMotor.setControl(flipperMotorTorqueCurrentRequest.withOutput(
                    FLIPPER_MOTOR_HOLD_OUT_CURRENT_AMPERES)
                );
            }
        }
    }

    public void incrementFlipperInwards() {
        switch (flipperPosition) {
            case OUT -> wantedState = INTAKE_STATE.INTAKE_POSITION_2;
            case POSITION_2 -> wantedState = INTAKE_STATE.INTAKE_POSITION_1;
            case POSITION_1 -> wantedState = INTAKE_STATE.INTAKE_IN_AND_OFF;
//            case OUT -> wantedState = INTAKE_STATE.INTAKE_POSITION_4;
//            case POSITION_4 -> wantedState = INTAKE_STATE.INTAKE_POSITION_3;
//            case POSITION_3 -> wantedState = INTAKE_STATE.INTAKE_POSITION_2;
//            case POSITION_2 -> wantedState = INTAKE_STATE.INTAKE_POSITION_1;
//            case POSITION_1 -> wantedState = INTAKE_STATE.INTAKE_IN_AND_OFF;
        }
    }

    public void resetFlipperOut() {
        wantedState = INTAKE_STATE.INTAKE_OUT_AND_ON;
    }

    public void setWantedState(INTAKE_STATE state) {
        this.wantedState = state;
    }

    public boolean isIntaking() { return (wantedState == INTAKE_STATE.INTAKE_IN_AND_OFF); }

    public boolean isOutaking() { return (wantedState == INTAKE_STATE.INTAKE_OUT_AND_ON); }

    /**
     * The position of the flipper. This is not just a number, since we are doing a custom current
     * based control, so the behavior is completely different for going in and out.
     */
    private enum FlipperPosition {
        IN,
        OUT,
        POSITION_1,
        POSITION_2
//        POSITION_3,
//        POSITION_4
    }

    public enum INTAKE_STATE {
        INTAKE_IN_AND_OFF(
            factory.getConstant(NAME, "intakeOffSpeed", 0, true),
            FlipperPosition.IN
        ),
        INTAKE_OUT_AND_ON(
            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
            FlipperPosition.OUT
        ),
        INTAKE_POSITION_1(
            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
            FlipperPosition.POSITION_1
        ),
        INTAKE_POSITION_2(
            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
            FlipperPosition.POSITION_2
        );
//        INTAKE_POSITION_3(
//            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
//            FlipperPosition.POSITION_3
//        ),
//        INTAKE_POSITION_4(
//            factory.getConstant(NAME, "intakeOnSpeed", .5, true),
//            FlipperPosition.POSITION_4
//        );

        private double intakeMotorValue;
        private final FlipperPosition flipperMotorPosition;

        INTAKE_STATE (double speed, FlipperPosition flipperMotorPosition) {
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
