package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Climber extends SubsystemBase implements ITestableSubsystem {

    public static final String NAME = "climber";
    private boolean wantedStateChanged;

    private CLIMBER_WANTED_STATE wantedState = CLIMBER_WANTED_STATE.STOW;

    public final Mechanism2d climberMech = new Mechanism2d(6, 6.5);
    public final MechanismRoot2d climberMechRoot = climberMech.getRoot("root", 1, 1.5);

    private double firstArmAngleDeg = 0;
    public final MechanismLigament2d climberMechFirstArm = climberMechRoot.append(new MechanismLigament2d("FirstArm", 3.5, firstArmAngleDeg));
    private double secondArmAngleDeg = -110;
    public final MechanismLigament2d climberMechSecondArm = climberMechFirstArm.append(new MechanismLigament2d("SecondArm", 1.5, secondArmAngleDeg));
    public final MechanismLigament2d climberMechThirdArm = climberMechSecondArm.append(new MechanismLigament2d("ThirdArm", 2, 50-secondArmAngleDeg));

    private final IMotor climbMotor = (IMotor)factory.getDevice(NAME, "climberMotor");
    PositionVoltage climbReq = new PositionVoltage(0);
    private final IMotor stowerMotor = (IMotor)factory.getDevice(NAME, "stowerMotor");
    PositionVoltage stowerReq = new PositionVoltage(0);

    double lastClimberMotorRequest = -10000;
    double lastStowerMotorRequest = -10000;

    private double climbCurrentPosition;
    private double stowerCurrentPosition;

    private double climbPositionStow = factory.getConstant(NAME, "climbPositionStow", 0);
    private double climbPositionReady = factory.getConstant(NAME, "climbPositionUnstow", 0);
    private double climbPositionL1 = factory.getConstant(NAME, "climbPositionL1", 0);
    private double climbPositionL2 = factory.getConstant(NAME, "climbPositionL2", 0);
    private double climbPositionL3 = factory.getConstant(NAME, "climbPositionL3", 0);

    private double stowerPositionStow = factory.getConstant(NAME, "stowerPositionStow", 0);
    private double stowerPositionUnstow = factory.getConstant(NAME, "stowerPositionUnstow", 0);

    private double climbReadyPositionTolerance = 0.01;
    private double stowerStowPositionTolerance = 0.01;

    public Climber () {
        super();
        SmartDashboard.putData("ClimbMechanism2d", climberMech);
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyState();
        outputToSmartDashboard();
    }

    @Override
    public void readFromHardware() {
        climbCurrentPosition = climbMotor.getMotorPosition();
        stowerCurrentPosition = stowerMotor.getMotorPosition();
    }

    private void applyState() {
        switch (wantedState) {
            case STOW -> coordinateControlRequest(climbPositionStow, stowerPositionStow);
            case READY -> coordinateControlRequest(climbPositionReady, stowerPositionUnstow);
            case L1 -> coordinateControlRequest(climbPositionL1, stowerPositionUnstow);
            case L2 -> coordinateControlRequest(climbPositionL2, stowerPositionUnstow);
            case L3 -> coordinateControlRequest(climbPositionL3, stowerPositionUnstow);
        }
    }

    private void coordinateControlRequest(double climbMotorReq, double stowerMotorReq){
        if (lastClimberMotorRequest != climbMotorReq) {
            climbMotor.setControl(climbReq.withPosition(climbMotorReq));
            lastClimberMotorRequest = climbMotorReq;
        }
        if (lastStowerMotorRequest != stowerMotorReq) {
            stowerMotor.setControl(stowerReq.withPosition(stowerMotorReq));
            lastStowerMotorRequest = stowerMotorReq;
        }
    }

    private void offsetClimbMotor(double offsetAmount){
        switch (wantedState) {
            case STOW -> climbPositionStow += offsetAmount;
            case READY -> climbPositionReady += offsetAmount;
            case L1 -> climbPositionL1 += offsetAmount;
            case L2 -> climbPositionL2 += offsetAmount;
            case L3 -> climbPositionL3 += offsetAmount;
        }
    }

    private void offsetStowerMotor(double offsetAmount){
        switch (wantedState) {
            case STOW -> stowerPositionStow += offsetAmount;
            case READY -> stowerPositionUnstow += offsetAmount;
/*            case L1 -> stowerPositionUnstow += offsetAmount;  //removed because this probably shouldn't be offset
            case L2 -> stowerPositionUnstow += offsetAmount;
            case L3 -> stowerPositionUnstow += offsetAmount;*/
        }
    }

    private void outputToSmartDashboard(){
        if (climberMechFirstArm.getAngle() != firstArmAngleDeg+90*(Math.abs(stowerCurrentPosition-stowerPositionStow)/Math.abs(stowerPositionUnstow-stowerPositionStow))) {
            climberMechFirstArm.setAngle(firstArmAngleDeg + 90 * (Math.abs(stowerCurrentPosition - stowerPositionStow) / Math.abs(stowerPositionUnstow - stowerPositionStow)));
            SmartDashboard.putData("ClimbMechanism2d", climberMech);
        }
    }

    public enum CLIMBER_WANTED_STATE {
        STOW,
        READY,
        L1,
        L2,
        L3
    }

    public enum CLIMBER_SYSTEM_STATE {
        STOW,
        READY,
        L1,
        L2,
        L3
    }

    public void setWantedState(CLIMBER_WANTED_STATE wantedState) {
        wantedStateChanged = true;
        this.wantedState = wantedState;
    }
}
