package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.ballisticCalc.BallisticCalculator;
import com.team1816.lib.ballisticCalc.BallisticSolution;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {
    String NAME = "shooter";

    private double GEAR_RATIO = 1.0;

    private double wantedAngle = 0.0;

    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");
    private final IMotor gatekeeperMotor = (IMotor) factory.getDevice(NAME, "shooterMotor");

    private VoltageOut voltageControl = new VoltageOut(0);
    private PositionVoltage positionControl = new PositionVoltage(0);
    private SHOOTER_STATE wantedState = SHOOTER_STATE.AUTOMATIC;
    private GATEKEEPER_STATE gatekeeperState = GATEKEEPER_STATE.CLOSED;

    private BallisticCalculator ballisticCalculator;
    private double targetX;
    private Translation3d targetTranslation;

    public double currentVoltage = 0;
    public double currentPosition = 0;

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

    public enum GATEKEEPER_STATE {
        OPEN,
        CLOSED
    }

    public void periodic() {
        readFromHardware();
        applyState();
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setWantedState(SHOOTER_STATE state, double angle) {
        this.wantedState = state;
        this.wantedAngle = angle;
    }

    @Override
    public void readFromHardware() {
        currentPosition = rotationAngleMotor.getMotorPosition();
        currentVoltage = 0;
    }

    private void applyState() {

        double launchAngle = wantedState.getLaunchAngle();
        double rotationAngle = wantedState.getRotationAngle();
        double launchVelocity = wantedState.getLaunchVelocity();

        if (wantedState == SHOOTER_STATE.AUTOMATIC) {
            // TODO: figure out launcher z value
            Translation3d launcherTranslation = new Translation3d(BaseRobotState.swerveDriveState.Pose.getX(), BaseRobotState.swerveDriveState.Pose.getY(), 22);

            // TODO: figure out hub z value
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                targetX = 4.6228;
            } else {
                targetX = 11.915394;
            }
            targetTranslation = new Translation3d(targetX, 03.8608, 40);
            BallisticSolution ballisticSolution = ballisticCalculator.getBallisticSolution(launcherTranslation, targetTranslation, 10);
            launchAngle = ballisticSolution.getLaunchAngle();
            rotationAngle = ballisticSolution.getRotationAngle();
            launchVelocity = ballisticSolution.getLaunchVelocity();
        }

        setShooterLaunchAngle(launchAngle);
        setShooterRotationAngle(rotationAngle);
        setShooterVelocity(launchVelocity);

        SmartDashboard.putString("Shooter state: ", wantedState.toString());
        SmartDashboard.putString("Gatekeeper state: ", gatekeeperState.toString());
    }

    private void setShooterVelocity(double wantedVelocity) {
        double output = MathUtil.clamp(wantedVelocity, -12.0, 12.0);

        SmartDashboard.putNumber("Shooter Voltage", output);

        topLaunchMotor.setControl(voltageControl.withOutput(output));
        bottomLaunchMotor.setControl(voltageControl.withOutput(output));
    }

    private void setShooterLaunchAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        rotationAngleMotor.setControl(positionControl.withPosition(rotations));
    }

    private void setShooterRotationAngle(double wantedAngle) {
        double rotations = (wantedAngle / 360.0) * GEAR_RATIO;

        rotationAngleMotor.setControl(positionControl.withPosition(rotations));
    }

    public void setWantedGatekeeperState(GATEKEEPER_STATE gatekeeperState) {
        this.gatekeeperState = gatekeeperState;
    }

}
