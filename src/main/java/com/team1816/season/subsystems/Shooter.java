package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.ballisticCalc.BallisticCalculator;
import com.team1816.lib.ballisticCalc.BallisticSolution;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {

    //CLASS
    String NAME = "shooter";

    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "launchAngleMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "rotationAngleMotor");

    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private PositionVoltage positionControl = new PositionVoltage(0);

    //AUTO AIM
    private BallisticCalculator ballisticCalculator = new BallisticCalculator();
    private AUTO_AIM_TARGETS currentTarget = AUTO_AIM_TARGETS.BLUE_HUB;
    private static final Translation3d SHOOTER_OFFSET = new Translation3d(0, 0, 22); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

    //CONSTANTS
    private static final double MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE = 3.5/1; //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final double MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE = 3.5/1; //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

    public enum AUTO_AIM_TARGETS{
        BLUE_HUB(new Translation3d(4.6228, 3.8608, 40)),
        RED_HUB(new Translation3d(11.915394, 3.8608, 40));

        private Translation3d position;

        AUTO_AIM_TARGETS (Translation3d position){
            this.position = position;
        }

        public Translation3d getPosition(){
            return position;
        }
    }

    public enum SHOOTER_STATE {
        DISTANCE_ONE(45, 0, 10),
        DISTANCE_TWO(45, 0, 20),
        DISTANCE_THREE(45, 0, 30),
        AUTOMATIC(-1, -1, -1),
        IDLE(0, 0, 0);

        private double launchAngle;
        private double rotationAngle;
        private double launchVelocity;
        SHOOTER_STATE (double launchAngle, double rotationAngle, double launchVelocity) {
            this.launchAngle = launchAngle;
            this.rotationAngle = rotationAngle;
            this.launchVelocity = launchVelocity;
        }

        double getLaunchAngle() {
            return launchAngle;
        }

        double getRotationAngle() {
            return rotationAngle;
        }

        double getLaunchVelocity() {
            return launchVelocity;
        }
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    @Override
    public void readFromHardware() {
    }

    private void applyState() {
        double launchAngle = wantedState.getLaunchAngle();
        double rotationAngle = wantedState.getRotationAngle();
        double launchVelocity = wantedState.getLaunchVelocity();

        if (wantedState == SHOOTER_STATE.AUTOMATIC) {
            Translation3d launcherTranslation = new Translation3d(BaseRobotState.swerveDriveState.Pose.getX(), BaseRobotState.swerveDriveState.Pose.getY(), 0).plus(SHOOTER_OFFSET);
            // TODO: figure out hub z value
            BallisticSolution ballisticSolution = ballisticCalculator.getBallisticSolution(launcherTranslation, currentTarget.getPosition(), 10);
            launchAngle = ballisticSolution.getLaunchAngle();
            rotationAngle = ballisticSolution.getRotationAngle();
            launchVelocity = ballisticSolution.getLaunchVelocity();
        }

        setLaunchAngle(launchAngle);
        setRotationAngle(rotationAngle);
        setVelocity(launchVelocity);

        SmartDashboard.putString("Shooter state: ", wantedState.toString());

        SmartDashboard.putNumber("Launch Angle: ", launchAngle);
        SmartDashboard.putNumber("Rotation Angle: ", rotationAngle);
        SmartDashboard.putNumber("Launch Velocity: ", launchVelocity);
    }

    public void setWantedState(SHOOTER_STATE state) {
        System.out.println("Setting Shooter to: "+state);
        this.wantedState = state;
    }

    public void setCurrentAutoAimTarget(AUTO_AIM_TARGETS target) {
        this.currentTarget = target;
    }

    private void setVelocity(double wantedVelocity) {
        double output = MathUtil.clamp(wantedVelocity, 0, 100);

        topLaunchMotor.setControl(velocityControl.withVelocity(output));
        bottomLaunchMotor.setControl(velocityControl.withVelocity(output));
    }

    private void setLaunchAngle(double wantedAngleDegrees) {
        double rotations = wantedAngleDegrees * MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE;

        double output = MathUtil.clamp(rotations, 0, 1000); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

        launchAngleMotor.setControl(positionControl.withPosition(rotations));
    }

    private void setRotationAngle(double wantedAngleDegrees) {
        double rotations = wantedAngleDegrees * MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE;

        double output = MathUtil.clamp(rotations, -100, 100); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

        rotationAngleMotor.setControl(positionControl.withPosition(rotations));
    }

}
