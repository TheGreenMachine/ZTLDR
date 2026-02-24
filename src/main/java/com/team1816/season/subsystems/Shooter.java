package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.ShooterDistanceSetting;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.ShooterTableCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "launchAngleMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "rotationAngleMotor");

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
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
    private final double CALIBRATION_THRESHOLD;
    private final Rotation2d CALIBRATION_POSITION_ARC_ANGLE;
    private final Rotation2d ROTATION_OFFSET_FROM_CALIBRATION_ZERO;
    private final double HALF_FIELD_WIDTH = FlippingUtil.fieldSizeY/2;
    private final double DISTANCE_BETWEEN_BEAM_BREAKS;
    private double leftLimit = 0;
    private double rightLimit = 0;

    //CALIBRATION
    private Double[] calibrationPositions = new Double[]{0.0, 0.0};
    private boolean isCalibrated;

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
        SNOWBLOWING(-1,-1, -1),
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

    public Shooter(){
        super();

        MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerLaunchAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerRotationAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        SHOOTER_OFFSET = new Translation3d(factory.getConstant(NAME, "initialShooterOffsetX",0), factory.getConstant(NAME, "initialShooterOffsetY",0), factory.getConstant(NAME, "initialShooterOffsetZ",0)); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        CALIBRATION_THRESHOLD = factory.getConstant(NAME, "calibrationThreshold",10); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        CALIBRATION_POSITION_ARC_ANGLE = Rotation2d.fromRotations(factory.getConstant(NAME, "calibrationPositionArcAngle", 0.75)); //should always be less than 1 rotation //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        ROTATION_OFFSET_FROM_CALIBRATION_ZERO = Rotation2d.fromDegrees(factory.getConstant(NAME, "rotationOffsetFromCalibrationZero", 70)); //as a note, the rotation motor should move clockwise on positive dutycycle, otherwise directions will be flipped //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        DISTANCE_BETWEEN_BEAM_BREAKS = factory.getConstant(NAME,"distanceBetweenBeamBreaks",0);

        launcherTranslation = new Translation3d(0,0,0).plus(SHOOTER_OFFSET);

        networkTable = NetworkTableInstance.getDefault().getTable("");
        turretFieldPose = networkTable.getDoubleArrayTopic("Field/Turret").publish();
        SmartDashboard.putData("Shooter Incline", launchMech);
        currentRotationPosition = rotationAngleMotor.getMotorPosition();
    }

    public void periodic() {
        SmartDashboard.putString("Shooter state: ", wantedState.toString());

        readFromHardware();

        if (wantedState == SHOOTER_STATE.CALIBRATING) {
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

        GreenLogger.log("Shooter state: " + wantedState.toString());
        GreenLogger.log("Launch Angle: " + launchAngle);
        GreenLogger.log("Launch Power: " + launchPower);
        GreenLogger.log("Rotation Angle: " + rotationAngle);
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setCurrentAutoAimTarget(AUTO_AIM_TARGETS target) {
        this.currentTarget = target;
    }

    private void calibratePeriodic(){
        // TODO: figure out which sensor value is a lower number of ticks
        if (!leftSensorValue) {
            double currentMotorPosition = rotationAngleMotor.getMotorPosition();
            leftLimit = currentMotorPosition;
            rightLimit = currentMotorPosition + DISTANCE_BETWEEN_BEAM_BREAKS;
            isCalibrated = true;
        }
        if (!rightSensorValue) {
            double currentMotorPosition = rotationAngleMotor.getMotorPosition();
            leftLimit = currentMotorPosition - DISTANCE_BETWEEN_BEAM_BREAKS;
            rightLimit = currentMotorPosition;
            isCalibrated = true;
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
        double rotations = (wantedAngleDegrees + ROTATION_OFFSET_FROM_CALIBRATION_ZERO.getDegrees()) * MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE;

        rotations = wrapMotorRotations(rotations);

        if(isCalibrated){
            // TODO: fix this based on if left or right is lower
            if (rotations < leftLimit || rotations > rightLimit){
                GreenLogger.log("Wanted Shooter rotation is out of bounds of the calibrated positions");
            }

            double output = MathUtil.clamp(rotations, leftLimit, rightLimit);

            rotationAngleMotor.setControl(positionControl.withPosition(output));
        } else {
            GreenLogger.log("Can't set rotation angle of shooter, rotation not calibrated");
        }
    }

    //this method technically causes inoptimal behavior in turret dead zones, but it shouldn't matter bc
    private double wrapMotorRotations(double rotations) {
        double scale = (calibrationPositions[1] - calibrationPositions[0]) / CALIBRATION_POSITION_ARC_ANGLE.getRotations();
        double scaledRotations = (rotations - calibrationPositions[0]) % scale;

        return scaledRotations + calibrationPositions[0];
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }
}
