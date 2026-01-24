package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;


/* TODO: 1. Add in Mech2d
 *   2. Add in shooter code
 *   3. Organize code layout
 *   4. Code in Incline an Turret Restrictions
 *   5. Set up offset code to find positions */

public class Shooter extends SubsystemBase implements ITestableSubsystem {
    String NAME = "shooter";
// TODO: Chnage Values
    private double GEAR_RATIO_TURRET = 1.0;
    private double GEAR_RATIO_INCLINE = 1.0;
    // todo: change give answers in m/s
    private double ExitVelocity = 0.0;

    private final IMotor turretMotor = (IMotor) factory.getDevice(NAME, "turretMotor");
    private final IMotor inclineMotor = (IMotor) factory.getDevice(NAME, "inclineMotor");
    private final IMotor shooterMotorLeader = (IMotor) factory.getDevice(NAME, "shooterMotorTop");
    private final IMotor shooterMotorFollower = (IMotor) factory.getDevice(NAME, "shooterMotorBottom");
    private final IMotor gatekeeperMotor = (IMotor) factory.getDevice(NAME, "gatekeeperMotor");

    private double gatekeeperSetUpSpeed = 0.7; //NUMBERS ARE TO CHANGE!!! + Unit is RPS
    private double gatekeeperRemovalSpeed = -.2;
    private double gatekeeperIdleSpeed = 0;

    private double shooterIdleSpeed = 0;
    private double shooterShootSpeed = 2;

    private VoltageOut voltageControl = new VoltageOut(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private VelocityVoltage velocityControl = new VelocityVoltage(0);

    private AIM_STATE wantedAimState = AIM_STATE.SHOOTER_IDLE;
    private GATEKEEPER_STATE wantedGatekeeperState = GATEKEEPER_STATE.IDLE;
    private SHOOTER_STATE wantedShooterState = SHOOTER_STATE.IDLE;
    public double currentVoltage = 0;

    public double currentTurretPosition = 0;
    public double curretGateKeeperSpeed = 0;
    public double currentInclinePosition = 0;
    double currentShooterVelocity = 0;


    public enum AIM_STATE {
        SCORING,
        SHOOT_RED,
        SHOOT_BLUE,
        SHOOTER_TO_INCLINE_3D,
        SHOOTER_ROTATE_180,
        SHOOTER_ROTATE_RIGHT,
        SHOOTER_IDLE,
        TEST_TURRET_180,
        SHOOT
    }
    public enum GATEKEEPER_STATE {
        IDLE,
        SET_UP,
        REMOVAL
    }
    public enum SHOOTER_STATE {
        IDLE,
        SHOOT //May need another shoot speed if max height in close ranges is too high
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedAimState(AIM_STATE state) {
        this.wantedAimState = state;
    }

    public void setWantedState(AIM_STATE state, double angle, double incline) {
        this.wantedAimState = state;

    }

    public void setWantedGatekeeperState(GATEKEEPER_STATE state) {
        this.wantedGatekeeperState = state;
    }

    @Override
    public void readFromHardware() {
        currentTurretPosition = turretMotor.getMotorPosition();
        curretGateKeeperSpeed = gatekeeperMotor.getMotorVelocity();
        currentInclinePosition = inclineMotor.getMotorPosition();
        currentShooterVelocity = shooterMotorLeader.getMotorVelocity();

        currentVoltage = 0;
    }

    private void applyState() {
        switch (wantedAimState) {
            case SHOOT_RED:
                setTurretAngle(getWantedAngleRedHub());
                setLaunchAngle(RobotPositionValues.getRedHypotonuse(),1.8288, ExitVelocity,9.8);
                break;
            case SHOOT_BLUE:
                setTurretAngle(getWantedAngleBlueHub());
                setLaunchAngle(RobotPositionValues.getBlueHypotonuse(),1.8288, ExitVelocity,9.8);
            break;
            case SHOOTER_TO_INCLINE_3D:

                break;
            case SHOOTER_IDLE:
            default:
                // TODO: Change These
                setTurretSpeed(0);
                setIncleSpeed(0);
                break;
            case TEST_TURRET_180:
                setTurretAngle(180);
                break;
        }

        switch (wantedGatekeeperState){
            case IDLE:
                setGatekeeperSpeed(gatekeeperIdleSpeed);
                break;
            case SET_UP:
                setGatekeeperSpeed(gatekeeperSetUpSpeed);
                break;
            case REMOVAL:
                setGatekeeperSpeed(gatekeeperRemovalSpeed);
                break;
            default:
                setGatekeeperSpeed(0);
                break;
        }

        switch (wantedShooterState) {
            case IDLE:
                setShooterSpeed(shooterIdleSpeed);
                break;
            case SHOOT:
                setShooterSpeed(shooterShootSpeed);
                break;
            default:
                setShooterSpeed(0);
                break;
        }
    }
    public double getWantedAngleBlueHub () {
        double wantedAngle = Math.acos(RobotPositionValues.getBlueRatios(RobotPositionValues.getBlueHypotonuse()));
        return wantedAngle;
    }
    public double getWantedAngleRedHub () {
        double wantedAngle = Math.acos(RobotPositionValues.getRedRatios(RobotPositionValues.getRedHypotonuse()));
        return wantedAngle;
    }

    public void setTurretSpeed(double wantedSpeed) {
        double output = MathUtil.clamp(wantedSpeed, -12.0, 12.0);

        SmartDashboard.putNumber("Shooter Voltage", output);

        turretMotor.setControl(voltageControl.withOutput(output));
    }

    public void setTurretAngle(double angle) {
        double rotations = (Math.toDegrees(angle) / 360.0) * GEAR_RATIO_TURRET;

        turretMotor.setControl(positionControl.withPosition(rotations));
    }
    public void setIncleSpeed(double wantedSpeed) {
        // TODO: Change Values
        double output = MathUtil.clamp(wantedSpeed, -12.0, 12.0);

        SmartDashboard.putNumber("Shooter Voltage", output);

        inclineMotor.setControl(voltageControl.withOutput(output));
    }
    public void setLaunchAngle (double Hypotonuse, double Alt, double Velocity, double Gravity){
        double angle = Math.atan(Math.pow(Velocity,2)+ Math.sqrt(Math.pow(Velocity,4)-Gravity * (Gravity * Math.pow(Hypotonuse,2) + 2 * (Alt - Velocity) * Math.pow(Velocity,2)))/(Gravity * Hypotonuse));
        double rotations = (Math.toDegrees(angle)/  360) * GEAR_RATIO_INCLINE;
        inclineMotor.setControl(positionControl.withPosition(rotations));
    }

    public void setGatekeeperSpeed(double wantedSpeed) {
        SmartDashboard.putNumber("Gatekeeper Velocity Voltage", wantedSpeed);

        gatekeeperMotor.setControl(velocityControl.withVelocity(wantedSpeed));
    }
    public void setShooterSpeed(double wantedSpeed) {
        SmartDashboard.putNumber("Shooter Velocity Voltage", wantedSpeed);

        shooterMotorLeader.setControl(velocityControl.withVelocity(wantedSpeed));
    }
}
