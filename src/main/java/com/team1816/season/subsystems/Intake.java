package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.Duration;
import java.time.Instant;

import static com.team1816.lib.Singleton.factory;

public class Intake extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "intake";

    private INTAKE_STATE wantedState = INTAKE_STATE.INTAKE_UP;

    //MOTORS
    private final IMotor intakeMotor = (IMotor) factory.getDevice(NAME, "intakeMotor");
    private final IMotor flipperMotor = (IMotor) factory.getDevice(NAME, "flipperMotor");

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    private static final double GEAR_RATIO = 1;


    //MECHANISMS
    public double currentPosition = 0;
    public Mechanism2d intakeMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    public MechanismRoot2d intakeMechRoot = intakeMech.getRoot("Intake Root", 0, 1);
    public MechanismLigament2d intakeAngleML = intakeMechRoot.append(
        new MechanismLigament2d("Intake Angle", 1.5, 0));

    public Intake () {
        super();
        SmartDashboard.putData("Intake", intakeMech);

        inSpeed = factory.getConstant(NAME, "inSpeed", -10, true);
        inAngle = factory.getConstant(NAME, "inAngle", 255, true);
        outSpeed = factory.getConstant(NAME, "outSpeed", 10, true);
        outAngle = factory.getConstant(NAME, "outAngle", 255, true);
        downAngle = factory.getConstant(NAME, "downAngle", 255, true);
        upAngle = factory.getConstant(NAME, "upAngle", 45, true);
    }

    private static double inSpeed = 0;
    private static double inAngle = 0;
    private static double outSpeed = 0;
    private static double outAngle = 0;
    private static double downAngle = 0;
    private static double upAngle = 0;

    public enum INTAKE_STATE {
        INTAKE_IN(
            inSpeed,
            inAngle
        ),
        INTAKE_OUT(
            outSpeed,
            outAngle
        ),
        INTAKE_DOWN(
            0,
            downAngle
        ),
        /// Acts as the idle
        INTAKE_UP(
            0,
            upAngle
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
        this.wantedState = state;
    }

    @Override
    public void readFromHardware() {
        currentPosition = intakeMotor.getMotorPosition();

        intakeAngleML.setAngle(wantedState.getAngle()); //todo: line up with actual subsystem & actual motor position
    }

    private void applyState() {
        double intakeSpeed = wantedState.getSpeed();
        double flipperAngle = wantedState.getAngle();

        setTurretSpeed(intakeSpeed);
        setFlipperAngle(flipperAngle);

        SmartDashboard.putString("Intake state: ", wantedState.toString());
        SmartDashboard.putNumber("Intake speed: ", intakeSpeed);
        SmartDashboard.putNumber("Flipper angle: ", flipperAngle);
    }

    public void setFlipperAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        rotations = MathUtil.clamp(rotations, -6000, 7000);

        flipperMotor.setControl(positionControl.withPosition(rotations));
    }

    public void setTurretSpeed(double wantedSpeed) {
        SmartDashboard.putNumber("Intake Velocity Voltage", wantedSpeed);

        wantedSpeed = MathUtil.clamp(wantedSpeed, -6000, 7000);

        intakeMotor.setControl(velocityControl.withVelocity(wantedSpeed));
    }
}
