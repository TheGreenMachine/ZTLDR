package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.MathUtil;
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

    private INTAKE_STATE wantedState = INTAKE_STATE.INTAKE_UP;
    private INTAKE_STATE previousWantedState = INTAKE_STATE.INTAKE_UP;

    //MOTORS
    private final IMotor intakeMotor;
    private final IMotor flipperMotor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MIN_INTAKE_MOTOR_CLAMP = -80;
    private static final double MAX_INTAKE_MOTOR_CLAMP = 80;
    private static final double MIN_FLIPPER_MOTOR_CLAMP = 0;
    private static final double MAX_FLIPPER_MOTOR_CLAMP = 80;

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
        intakeMotor = (IMotor) factory.getDevice(NAME, "intakeMotor");
        flipperMotor = (IMotor) factory.getDevice(NAME, "flipperMotor");
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

    public void adjustCurrentFlipperMotorSetPoint(double adjustValue){
        wantedState.adjustFlipperMotorSetPoint(adjustValue);

        GreenLogger.log(NAME+"/flipperMotor/"+wantedState.name()+"_adjusted_value/"+wantedState.getFlipperMotorValue());
    }

    @Override
    public void readFromHardware() {
        currentPosition = flipperMotor.getMotorPosition();

        intakeAngleML.setAngle(currentPosition * ROTATIONS_TO_DEGREES_RATIO);
    }

    private void applyState() {
        double intakeSpeed = wantedState.getIntakeMotorValue();
        double flipperAngle = wantedState.getFlipperMotorValue();

        setIntakeSpeed(wantedState.getIntakeMotorValue());
        setFlipperAngle(wantedState.getFlipperMotorValue());

        if (wantedState != previousWantedState) {
            GreenLogger.log("Intake state: " + wantedState.toString());
            GreenLogger.log("Intake speed: " + intakeSpeed);
            GreenLogger.log("Intake angle: " + flipperAngle);

            SmartDashboard.putString("Intake state: ", wantedState.toString());
            SmartDashboard.putNumber("Intake speed: ", intakeSpeed);
            SmartDashboard.putNumber("Intake angle: ", flipperAngle);
            previousWantedState = wantedState;
        }
    }

    private void setIntakeSpeed(double velocity) {
        double clampedVelocity = MathUtil.clamp(velocity, MIN_INTAKE_MOTOR_CLAMP, MAX_INTAKE_MOTOR_CLAMP);

        intakeMotor.setControl(velocityControl.withVelocity(clampedVelocity));
    }

    private void setFlipperAngle(double rotations) {
        double clampRotation = MathUtil.clamp(rotations, MIN_FLIPPER_MOTOR_CLAMP, MAX_FLIPPER_MOTOR_CLAMP);

        flipperMotor.setControl(positionControl.withPosition(clampRotation));
    }

    public void setWantedState(INTAKE_STATE state) {
        this.wantedState = state;
    }

    public boolean isIntaking() { return (wantedState == INTAKE_STATE.INTAKE_IN); }

    public boolean isOutaking() { return (wantedState == INTAKE_STATE.INTAKE_OUT); }

    public enum INTAKE_STATE {
        INTAKE_IN(factory.getConstant(NAME, "inSpeed", -10, true), factory.getConstant(NAME, "inAngle", 255, true)),
        INTAKE_OUT(factory.getConstant(NAME, "outSpeed", 10, true), factory.getConstant(NAME, "outAngle", 255, true)),
        INTAKE_DOWN(0, factory.getConstant(NAME, "downAngle", 255, true)),
        // Acts as the idle
        INTAKE_UP(0, factory.getConstant(NAME, "upAngle", 45, true));

        private double intakeMotorValue, flipperMotorValue;

        INTAKE_STATE (double speed, double flipperMotorValue) {
            this.intakeMotorValue = speed;
            this.flipperMotorValue = flipperMotorValue;
        }

        private void adjustIntakeMotorSetPoint(double adjustValue){
            this.intakeMotorValue += adjustValue;
        }

        private void adjustFlipperMotorSetPoint(double adjustValue){
            this.flipperMotorValue += adjustValue;
        }

        public double getIntakeMotorValue() {
            return intakeMotorValue;
        }

        public double getFlipperMotorValue() {
            return flipperMotorValue;
        }
    }
}
