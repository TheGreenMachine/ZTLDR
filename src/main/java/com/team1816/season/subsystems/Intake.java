package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.GreenLogger;
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
    }

    public enum INTAKE_STATE {
        INTAKE_IN(
            factory.getConstant(NAME, "inSpeed", -10, true),
            factory.getConstant(NAME, "inAngle", 255, true)
        ),
        INTAKE_OUT(
            factory.getConstant(NAME, "outSpeed", 10, true),
            factory.getConstant(NAME, "outAngle", 255, true)
        ),
        INTAKE_DOWN(
            0,
            factory.getConstant(NAME, "downAngle", 255, true)
        ),
        /// Acts as the idle
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
        this.wantedState = state;
    }

    @Override
    public void readFromHardware() {
        currentPosition = intakeMotor.getMotorPosition();

        intakeAngleML.setAngle(wantedState.getAngle());
    }

    private void applyState() {
        double intakeSpeed = wantedState.getSpeed();
        double flipperAngle = wantedState.getAngle();

        setTurretSpeed(intakeSpeed);
        setFlipperAngle(flipperAngle);

        GreenLogger.log("Intake state: " + wantedState.toString());
        GreenLogger.log("Intake speed: " + intakeSpeed);
        GreenLogger.log("Intake angle: " + flipperAngle);
    }

    public void setFlipperAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        rotations = MathUtil.clamp(rotations, -6000, 7000);

        flipperMotor.setControl(positionControl.withPosition(rotations));
    }

    public void setTurretSpeed(double wantedSpeed) {
        wantedSpeed = MathUtil.clamp(wantedSpeed, -6000, 7000);

        intakeMotor.setControl(velocityControl.withVelocity(wantedSpeed));
    }
}
