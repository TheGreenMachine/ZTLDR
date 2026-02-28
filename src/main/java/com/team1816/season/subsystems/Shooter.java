package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.ShooterDistanceSetting;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.ShooterTableCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {

    //CLASS
    public static final String NAME = "shooter";

    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE previousWantedState = SHOOTER_STATE.IDLE;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "launchAngleMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "rotationAngleMotor");

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private VelocityVoltage rotationVelocityControl = new VelocityVoltage(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    //AUTO AIM
    private AUTO_AIM_TARGETS currentTarget = AUTO_AIM_TARGETS.RED_HUB;
    // TODO: get the launcher position from the vision or whatever
    private Translation3d launcherTranslation;

    //DEVICES
    private final DigitalInput rotationAngleSensorClockwiseLeft = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseLeft", 0));
    private final DigitalInput rotationAngleSensorClockwiseRight = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseRight", 0));

    //HARDWARE RECORDED VALUES
    private double currentRotationPosition;
    private boolean leftSensorValue = true;
    private boolean rightSensorValue = true;

    //CONSTANTS
    private final double MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE;
    private final double MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE;
    private final Translation3d SHOOTER_OFFSET;
    private final double HALF_FIELD_WIDTH = FlippingUtil.fieldSizeY/2;

    //CALIBRATION
    private final double FAST_CALIBRATION_SPEED = .2;
    private final double SLOW_CALIBRATION_SPEED = .1;
    private final double EXTRA_SLOW_CALIBRATION_SPEED = .05;
    private final double CALIBRATION_STALL_SECONDS = .5;
    private final double ONE_TURRET_ROTATION = 1;
    private double initialCalibrationTimestamp = -1;
    private CALIBRATION_STATE calibrationState;
    private double rotationAngleMotorOffsetRotations;
    private final DutyCycleOut dutyCycleOutRequest = new DutyCycleOut(0);

    //MECHANISMS
    private final NetworkTable networkTable;
    private final DoubleArrayPublisher turretFieldPose;
    private final double[] poseArray = new double[3];

    private Mechanism2d launchMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    private MechanismRoot2d launchMechRoot = launchMech.getRoot("Launch Root", 1.5, 0);
    private MechanismLigament2d launchAngleML = launchMechRoot.append(
        new MechanismLigament2d("Launch Angle", 1.5, 0));

    public enum AUTO_AIM_TARGETS{
        // TODO: figure out hub z value
        BLUE_HUB(new Translation3d(4.6228, 3.8608, 40)),
        RED_HUB(new Translation3d(11.915394, 3.8608, 40)),
        BLUE_LEFT_CORNER(new Translation3d(2, 6.07, 0)),
        BLUE_RIGHT_CORNER(new Translation3d(2, 2, 0)),
        RED_LEFT_CORNER(new Translation3d(16.27, 2, 0)),
        RED_RIGHT_CORNER(new Translation3d(16.27, 6.07, 0));

        private Translation3d position;

        AUTO_AIM_TARGETS (Translation3d position){
            this.position = position;
        }

        public Translation3d getPosition(){
            return position;
        }
    }

    private ShooterTableCalculator shooterTableCalculator = new ShooterTableCalculator();

    public enum SHOOTER_STATE {
        // TODO: figure out what default angles and velocities should be for manual mode
        CALIBRATING(0, 0, 0),
        DISTANCE_ONE(factory.getConstant(NAME,"distanceOneLaunchAngle",0), factory.getConstant(NAME,"distanceOneRotationAngle",0), factory.getConstant(NAME,"distanceOneLaunchVelocity",0)),
        DISTANCE_TWO(factory.getConstant(NAME,"distanceTwoLaunchAngle",0), factory.getConstant(NAME,"distanceTwoRotationAngle",0), factory.getConstant(NAME,"distanceTwoLaunchVelocity",0)),
        DISTANCE_THREE(factory.getConstant(NAME,"distanceThreeLaunchAngle",0), factory.getConstant(NAME,"distanceThreeRotationAngle",0), factory.getConstant(NAME,"distanceThreeLaunchVelocity",0)),
        AUTOMATIC(-1, -1, -1),
        SNOWBLOWING(-1, -1, -1),
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

        double getLaunchPower() {
            return launchVelocity;
        }
    }

    public enum CALIBRATION_STATE {
        CALIBRATING,
        CALIBRATED
    }

    public Shooter(){
        super();

        MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerLaunchAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerRotationAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        SHOOTER_OFFSET = new Translation3d(factory.getConstant(NAME, "initialShooterOffsetX",0), factory.getConstant(NAME, "initialShooterOffsetY",0), factory.getConstant(NAME, "initialShooterOffsetZ",0)); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

        launcherTranslation = new Translation3d(0,0,0).plus(SHOOTER_OFFSET);

        calibrationState = CALIBRATION_STATE.CALIBRATING;

        networkTable = NetworkTableInstance.getDefault().getTable("");
        turretFieldPose = networkTable.getDoubleArrayTopic("Field/Turret").publish();
        SmartDashboard.putData("Shooter Incline", launchMech);
        currentRotationPosition = rotationAngleMotor.getMotorPosition();
    }

    public void periodic() {
        readFromHardware();

        if (rotationAngleMotor.hasDeviceCrashed()) {
            calibrationState = CALIBRATION_STATE.CALIBRATING;
        }

        if (!(calibrationState == CALIBRATION_STATE.CALIBRATED)) {
            calibratePeriodic();
        } else {
            applyState();
        }
    }

    @Override
    public void readFromHardware() {
        currentRotationPosition = rotationAngleMotor.getMotorPosition();

        leftSensorValue = rotationAngleSensorClockwiseLeft.get();
        rightSensorValue = rotationAngleSensorClockwiseRight.get();

        launcherTranslation = new Translation3d(BaseRobotState.swerveDriveState.Pose.getX(), BaseRobotState.swerveDriveState.Pose.getY(), 0).plus(SHOOTER_OFFSET);

        var robotPose = BaseRobotState.swerveDriveState.Pose;
        poseArray[0] = robotPose.getX();
        poseArray[1] = robotPose.getY();
        poseArray[2] = rotationAngleMotor.getMotorPosition() / MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE;
        turretFieldPose.set(poseArray);

        launchAngleML.setAngle(wantedState.getLaunchAngle()); //todo: Will need to change to correspond with motor
    }

    private void applyState() {
        double launchAngle = wantedState.getLaunchAngle();
        double rotationAngle = wantedState.getRotationAngle();
        double launchPower = wantedState.getLaunchPower();

        if (wantedState == SHOOTER_STATE.AUTOMATIC || wantedState == SHOOTER_STATE.SNOWBLOWING) {

            if (wantedState == SHOOTER_STATE.AUTOMATIC) {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                    setCurrentAutoAimTarget(AUTO_AIM_TARGETS.RED_HUB);
                }
                else {
                    setCurrentAutoAimTarget(AUTO_AIM_TARGETS.BLUE_HUB);
                }
            }
            else {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
                        setCurrentAutoAimTarget(AUTO_AIM_TARGETS.RED_RIGHT_CORNER);
                    }
                    else {
                        setCurrentAutoAimTarget(AUTO_AIM_TARGETS.RED_LEFT_CORNER);
                    }
                } else {
                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
                        setCurrentAutoAimTarget(AUTO_AIM_TARGETS.BLUE_LEFT_CORNER);
                    }
                    else {
                        setCurrentAutoAimTarget(AUTO_AIM_TARGETS.BLUE_RIGHT_CORNER);
                    }
                }
            }

            double distance = launcherTranslation.toTranslation2d().getDistance(currentTarget.position.toTranslation2d());

            ShooterDistanceSetting shooterDistanceSetting = shooterTableCalculator.getShooterDistanceSetting(distance);
            launchAngle = shooterDistanceSetting.getAngle();
            launchPower = shooterDistanceSetting.getPower();
            rotationAngle = Math.tan((launcherTranslation.getY()-currentTarget.position.getY())/(launcherTranslation.getX()-currentTarget.position.getX()));
        }

        setLaunchAngle(launchAngle);
        setRotationAngle(rotationAngle);
        setPower(launchPower);

        if (wantedState != previousWantedState) {
            GreenLogger.log("Shooter state: " + wantedState.toString());
            GreenLogger.log("Launch Angle: " + launchAngle);
            GreenLogger.log("Launch Power: " + launchPower);
            GreenLogger.log("Rotation Angle: " + rotationAngle);

            SmartDashboard.putString("Shooter state: ", wantedState.toString());
            SmartDashboard.putNumber("Launch Angle: ", launchAngle);
            SmartDashboard.putNumber("Launch Power: ", launchPower);
            SmartDashboard.putNumber("Rotation Angle: ", rotationAngle);
            previousWantedState = wantedState;
        }
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setCurrentAutoAimTarget(AUTO_AIM_TARGETS target) {
        this.currentTarget = target;
    }

    private void calibratePeriodic(){
        // TODO: figure out if left, right, and clockwise are used correctly
        if (calibrationState == CALIBRATION_STATE.CALIBRATING) {
            if (!rightSensorValue && !leftSensorValue) {
                rotationAngleMotor.setControl(dutyCycleOutRequest.withOutput(EXTRA_SLOW_CALIBRATION_SPEED));
                if (initialCalibrationTimestamp == -1) {
                    initialCalibrationTimestamp = Timer.getFPGATimestamp();
                }
                else if (Timer.getFPGATimestamp() - initialCalibrationTimestamp > CALIBRATION_STALL_SECONDS) {
                    rotationAngleMotorOffsetRotations = rotationAngleMotor.getMotorPosition();
                    calibrationState = CALIBRATION_STATE.CALIBRATED;
                }
            }
            else {
                initialCalibrationTimestamp = -1;
                rotationAngleMotor.setControl(dutyCycleOutRequest.withOutput(leftSensorValue ? FAST_CALIBRATION_SPEED : SLOW_CALIBRATION_SPEED));
            }
        }
    }

    private void setAutomaticRotationAngle(){
        if(wantedState != SHOOTER_STATE.AUTOMATIC) return;

        Rotation2d robotCentricWantedAngle =
            currentTarget.getPosition().toTranslation2d()
                .minus(BaseRobotState.swerveDriveState.Pose.getTranslation())
                .getAngle()
                .minus(BaseRobotState.swerveDriveState.Pose.getRotation());
    }

    private void setPower(double wantedVelocity) {
        double output = MathUtil.clamp(wantedVelocity, 0, 100);

        topLaunchMotor.setControl(velocityControl.withVelocity(output));
        bottomLaunchMotor.setControl(velocityControl.withVelocity(output));
    }

    private void setLaunchAngle(double wantedAngleDegrees) {
        double rotations = wantedAngleDegrees * MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE;

        double output = MathUtil.clamp(rotations, 0, 1000); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

        launchAngleMotor.setControl(positionControl.withPosition(output));
    }

    private void setRotationAngle(double wantedAngleDegrees) {
        double rotations = wantedAngleDegrees  * MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE + rotationAngleMotorOffsetRotations;

        rotations = wrapMotorRotations(rotations);

        if(calibrationState == CALIBRATION_STATE.CALIBRATED){
            // TODO: fix this based on if left or right is lower
            if (rotations < rotationAngleMotorOffsetRotations || rotations > rotationAngleMotorOffsetRotations + ONE_TURRET_ROTATION){
                GreenLogger.log("Wanted Shooter rotation is out of bounds of the calibrated positions");
            }

            double output = MathUtil.clamp(rotations, rotationAngleMotorOffsetRotations, rotationAngleMotorOffsetRotations + ONE_TURRET_ROTATION);

            rotationAngleMotor.setControl(positionControl.withPosition(output));
        } else {
            GreenLogger.log("Can't set rotation angle of shooter, rotation not calibrated");
        }
    }

    //this method technically causes inoptimal behavior in turret dead zones, but it shouldn't matter bc
    private double wrapMotorRotations(double rotations) {
        double scaledRotations = (rotations - rotationAngleMotorOffsetRotations) % ONE_TURRET_ROTATION;

        return scaledRotations + rotationAngleMotorOffsetRotations;
    }

    public boolean isCalibrated() {
        return calibrationState == CALIBRATION_STATE.CALIBRATED;
    }
}
