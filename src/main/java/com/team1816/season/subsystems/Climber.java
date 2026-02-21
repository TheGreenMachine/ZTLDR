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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {
    //CLASS
    public static final String NAME = "climber";

    private CLIMBER_STATE wantedState = CLIMBER_STATE.IDLING;

    //MOTORS
    private final IMotor flipMotor = (IMotor)factory.getDevice(NAME, "flipMotor");
    private final IMotor linearSlideMotor = (IMotor)factory.getDevice(NAME, "linearSlideMotor");

    private final PositionVoltage positionReq = new PositionVoltage(0);

    //YAML VALUES
    private double flipIdling = factory.getConstant(NAME, "flipIdling", 0);
    private double flipL3UpClimbing = factory.getConstant(NAME, "flipL3UpClimbing", 0);
    private double flipL3DownClimbing = factory.getConstant(NAME, "flipL3DownClimbing", 0);
    private double flipL1UpClimbing = factory.getConstant(NAME, "flipL1UpClimbing", 0);
    private double flipL1DownClimbing = factory.getConstant(NAME, "flipL1DownClimbing", 0);

    private double linearSlideIdling = factory.getConstant(NAME, "linearSlideIdling", 0);
    private double linearSlideL3UpClimbing = factory.getConstant(NAME, "linearSlideL3UpClimbing", 0);
    private double linearSlideL3DownClimbing = factory.getConstant(NAME, "linearSlideL3DownClimbing", 0);
    private double linearSlideL1UpClimbing = factory.getConstant(NAME, "linearSlideL1UpClimbing", 0);
    private double linearSlideL1DownClimbing = factory.getConstant(NAME, "linearSlideL1DownClimbing", 0);

    //PHYSICAL SUBSYSTEM DEPENDENT CONSTANTS
    private static final double MAX_FLIP_MOTOR_CLAMP = 100;
    private static final double MIN_FLIP_MOTOR_CLAMP = 0;
    private static final double MAX_LINEAR_SLIDE_MOTOR_CLAMP = 100;
    private static final double MIN_LINEAR_SLIDE_MOTOR_CLAMP = 0;

    //MECHANISMS
    private double linearSlidePosition;
    private final Mechanism2d climberMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    private final MechanismRoot2d climberMechRoot = climberMech.getRoot("Climber Root", 1,0);
    private final MechanismLigament2d climberArm = climberMechRoot.append(new MechanismLigament2d("Climber Lift", 1, 70));
    private final MechanismLigament2d linearSlide = climberMechRoot.append(new MechanismLigament2d("Linear Slide", 0, 0, 10, new Color8Bit(Color.kPurple)));

    public Climber () {
        super();
        SmartDashboard.putData("Climber", climberMech);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
        linearSlidePosition = linearSlideMotor.getMotorPosition();
        linearSlide.setLength(linearSlidePosition/(MAX_LINEAR_SLIDE_MOTOR_CLAMP-MIN_LINEAR_SLIDE_MOTOR_CLAMP));
    }

    private void applyState() {
        switch (wantedState) {
            case IDLING:
                setFlipMotor(flipIdling);
                setLinearSlideMotor(linearSlideIdling);
                break;
            case L3_UP_CLIMBING:
                setFlipMotor(flipL3UpClimbing);
                setLinearSlideMotor(linearSlideL3UpClimbing);
                break;
            case L3_DOWN_CLIMBING:
                setFlipMotor(flipL3DownClimbing);
                setLinearSlideMotor(linearSlideL3DownClimbing);
                break;
            case L1_UP_CLIMBING:
                setFlipMotor(flipL1UpClimbing);
                setLinearSlideMotor(linearSlideL1UpClimbing);
                break;
            case L1_DOWN_CLIMBING:
                setFlipMotor(flipL1DownClimbing);
                setLinearSlideMotor(linearSlideL1DownClimbing);
                break;
            default:
                GreenLogger.log("Climber state: "+wantedState+" is not accounted for in applyState()");
                break;
        }

        SmartDashboard.putString("Climber state: ", wantedState.toString());
    }

    private void setFlipMotor(double position){
        double clampedPosition = MathUtil.clamp(position, MIN_FLIP_MOTOR_CLAMP, MAX_FLIP_MOTOR_CLAMP);

        flipMotor.setControl(positionReq.withPosition(clampedPosition));
    }

    private void setLinearSlideMotor(double position){
        double clampedPosition = MathUtil.clamp(position, MIN_LINEAR_SLIDE_MOTOR_CLAMP, MAX_LINEAR_SLIDE_MOTOR_CLAMP);

        linearSlideMotor.setControl(positionReq.withPosition(clampedPosition));
    }

    public enum CLIMBER_STATE {
        IDLING,
        L3_UP_CLIMBING,
        L3_DOWN_CLIMBING,
        L1_UP_CLIMBING,
        L1_DOWN_CLIMBING
    }

    public void setWantedState(CLIMBER_STATE wantedState) {
        this.wantedState = wantedState;
    }
}
