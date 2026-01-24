package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;

public class Shooter extends SubsystemBase implements ITestableSubsystem {
    /*TODO: 1. create "findAngle" method
    2. Code in blind spot
    3. create "findIncline" method
    4. set up state machine "gatekeeper" motor (set up, idle, stop) *system state might be needed
    5. Write in the 2 "shooter" motors
    6. Mech 2d implementation
    7. Servo implementation
    8. Beam break implementation
    9. 17.5:1 rot:cycle
    10. 1:22.5  rot: degree
    11. Added subsystem to yaml ✓(for now)
    */
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




    private VoltageOut voltageControl = new VoltageOut(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private SHOOTER_STATE wantedState = SHOOTER_STATE.SHOOTER_IDLE;
    public double currentVoltage = 0;
    public double currentPosition = 0;


    public enum SHOOTER_STATE {
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
//    public enum INCLINE_STATE {
//        LOW,
//        MID,
//        HIGH
//    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setWantedState(SHOOTER_STATE state, double angle, double incline) {
        this.wantedState = state;

    }

    @Override
    public void readFromHardware() {
        currentPosition = turretMotor.getMotorPosition();
        currentVoltage = 0;
    }

    private void applyState() {
        switch (wantedState) {
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
//            case SHOOTER_TO_ANGLE:
//                setTurretAngle(wantedAngle);
//                break;
//            case SHOOTER_ROTATE_LEFT:
//                setTurretSpeed(1);
//                break;
//            case SHOOTER_ROTATE_RIGHT:
//                setTurretSpeed(-1);
//                break;
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
}
