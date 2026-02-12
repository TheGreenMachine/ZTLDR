package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private final IMotor climberFlipMotor = (IMotor)factory.getDevice(NAME, "climberFlipMotor");
    private final IMotor linearMotor = (IMotor)factory.getDevice(NAME, "linearMotor"); //Build-team had a cool name for this
    private double curPosition;
    private CLIMBER_STATE wantedState = CLIMBER_STATE.IDLING;
    VelocityVoltage climReq = new VelocityVoltage(0);

    //MECHANISMS *Need to ask build team for details
    public Mechanism2d climberMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    public MechanismRoot2d climberMechRoot = climberMech.getRoot("Climber Root", 1.5, 0);
    public MechanismLigament2d climberAngleML = climberMechRoot.append(
        new MechanismLigament2d("Climber Angle", 1, 0));

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
        curPosition = climberFlipMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case IDLING:

                break;
            case L3_CLIMBING:

                break;
            case L3_DOWN_CLIMBING:

                break;
            case L1_CLIMBING:

                break;
            case L1_DOWN_CLIMBING:

                break;
            default:
                break;
        }

        SmartDashboard.putString("Climber state: ", wantedState.toString());
        climberAngleML.setAngle(0); //TO ASK
    }

    public enum CLIMBER_STATE {
        IDLING,
        L3_CLIMBING,
        L3_DOWN_CLIMBING,
        L1_CLIMBING,
        L1_DOWN_CLIMBING


    }

    public void setWantedState(CLIMBER_STATE wantedState) {
        this.wantedState = wantedState;
    }
}
