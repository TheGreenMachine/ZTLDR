package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private final IMotor climberFlipMotor = (IMotor) factory.getDevice(NAME, "climberFlipMotor");
    private final IMotor linearMotor = (IMotor) factory.getDevice(NAME, "linearMotor"); //Build-team had a cool name for this
    VelocityVoltage climReq = new VelocityVoltage(0);
    private double curPosition;
    private CLIMBER_STATE wantedState = CLIMBER_STATE.IDLING;

    //MECHANISMS *Need to ask build team for details
    public Mechanism2d climberMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    public MechanismRoot2d climberMechRoot = climberMech.getRoot("Climber Root", 1.5,0);
    public MechanismLigament2d climberLiftML = climberMechRoot.append(
        new MechanismLigament2d("Climber Lift", 1, 90));

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
        climberLiftML.setLength(wantedState.getLength());
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
    }

    public void setWantedState(CLIMBER_STATE wantedState) {
        this.wantedState = wantedState;
    }

    public enum CLIMBER_STATE {
        IDLING(
            factory.getConstant(NAME, "idlePosition", 0, true)
        ),
        L3_CLIMBING(
            factory.getConstant(NAME, "L3ClimbingPosition", 10, true)
        ),
        L3_DOWN_CLIMBING(
            factory.getConstant(NAME, "L3ClimbingDownPosition", 0, true)
        ),
        L1_CLIMBING(
            factory.getConstant(NAME, "L1ClimbingPosition", 10, true)
        ),
        L1_DOWN_CLIMBING(
            factory.getConstant(NAME, "L1ClimbingDownPosition", 0, true)
        );


        private final double length;
        CLIMBER_STATE (double length) {
            this.length = length;
        }

        public double getLength() {
            return length;
        }
    }
}
