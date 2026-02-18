package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {

    //CLASS
    private static final String NAME = "shooter";

    // Always default this to IDLE, the real default for the shooter is in the superstructure
    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;

    private boolean isCalibrating;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "launchAngleMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "rotationAngleMotor");

    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private PositionVoltage positionControl = new PositionVoltage(0);

    //AUTO AIM
    private AUTO_AIM_TARGETS currentTarget = AUTO_AIM_TARGETS.BLUE_HUB;
    // TODO: get the launcher position from the vision or whatever
    Translation3d launcherTranslation = new Translation3d(0,0,0).plus(SHOOTER_OFFSET);

    //DEVICES
    private final DigitalInput rotationAngleSensorClockwiseLeft = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseLeft", 0));
    private final DigitalInput rotationAngleSensorClockwiseRight = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseRight", 0));

    //HARDWARE RECORDED VALUES
    double currentRotationPosition;
    boolean leftSensorValue = true;
    boolean rightSensorValue = true;

    //CONSTANTS
    private static final double MOTOR_ROTATIONS_PER_LAUNCH_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerLaunchAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final double MOTOR_ROTATIONS_PER_ROTATION_ANGLE_DEGREE = factory.getConstant(NAME, "motorRotationsPerRotationAngleDegree", 0); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final Translation3d SHOOTER_OFFSET = new Translation3d(factory.getConstant(NAME, "initialShooterOffsetX",0), factory.getConstant(NAME, "initialShooterOffsetY",0), factory.getConstant(NAME, "initialShooterOffsetZ",0)); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final double CALIBRATION_THRESHOLD = factory.getConstant(NAME, "calibrationThreshold",10); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final Rotation2d CALIBRATION_POSITION_ARC_ANGLE = Rotation2d.fromRotations(factory.getConstant(NAME, "calibrationThreshold", 0.75)); //should always be less than 1 rotation //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
    private static final Rotation2d ROTATION_OFFSET_FROM_CALIBRATION_ZERO = Rotation2d.fromDegrees(factory.getConstant(NAME, "rotationOffsetFromCalibrationZero", 70)); //as a note, the rotation motor should move clockwise on positive dutycycle, otherwise directions will be flipped //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.

    //CALIBRATION
    private Double[] calibrationPositions = new Double[]{0.0, 0.0};

    //MECHANISMS
    private final NetworkTable networkTable;
    private DoubleArrayPublisher turretFieldPose;
    private final double[] poseArray = new double[3];

    public Mechanism2d launchMech = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    public MechanismRoot2d launchMechRoot = launchMech.getRoot("Launch Root", 1.5, 0);
    public MechanismLigament2d launchAngleML = launchMechRoot.append(
        new MechanismLigament2d("Launch Angle", 1.5, 0));

    public enum AUTO_AIM_TARGETS{
        // TODO: figure out hub z value
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

    private ShooterTableCalculator shooterTableCalculator = new ShooterTableCalculator();

    public enum SHOOTER_STATE {
        // TODO: figure out what default angles and velocities should be for manual mode
        DISTANCE_ONE(factory.getConstant(NAME,"distanceOneLaunchAngle",0), factory.getConstant(NAME,"distanceOneRotationAngle",0), factory.getConstant(NAME,"distanceOneLaunchVelocity",0)),
        DISTANCE_TWO(factory.getConstant(NAME,"distanceTwoLaunchAngle",0), factory.getConstant(NAME,"distanceTwoRotationAngle",0), factory.getConstant(NAME,"distanceTwoLaunchVelocity",0)),
        DISTANCE_THREE(factory.getConstant(NAME,"distanceThreeLaunchAngle",0), factory.getConstant(NAME,"distanceThreeRotationAngle",0), factory.getConstant(NAME,"distanceThreeLaunchVelocity",0)),
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

        double getLaunchPower() {
            return launchVelocity;
        }
    }

    public Shooter(){
        super();
        networkTable = NetworkTableInstance.getDefault().getTable("");
        turretFieldPose = networkTable.getDoubleArrayTopic("Field/Turret").publish();
        SmartDashboard.putData("Shooter Incline", launchMech);
        currentRotationPosition = rotationAngleMotor.getMotorPosition();
    }

    public void periodic() {
        SmartDashboard.putString("Is Calibrating: ", "" + isCalibrating);
        SmartDashboard.putString("Shooter state: ", wantedState.toString());

        readFromHardware();

        if (isCalibrating) {
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

        if (wantedState == SHOOTER_STATE.AUTOMATIC) {
            double distance = launcherTranslation.toTranslation2d().getDistance(currentTarget.position.toTranslation2d());

            ShooterDistanceSetting shooterDistanceSetting = shooterTableCalculator.getShooterDistanceSetting(distance);
            launchAngle = shooterDistanceSetting.getAngle();
            launchPower = shooterDistanceSetting.getPower();
            rotationAngle = Math.tan((launcherTranslation.getY()-currentTarget.position.getY())/(launcherTranslation.getX()-currentTarget.position.getX()));
        }

        setLaunchAngle(launchAngle);
        setRotationAngle(rotationAngle);
        setPower(launchPower);

        SmartDashboard.putString("Shooter state: ", wantedState.toString());
        SmartDashboard.putNumber("Launch Angle: ", launchAngle);
        SmartDashboard.putNumber("Launch Power: ", launchPower);
        SmartDashboard.putNumber("Rotation Angle: ", rotationAngle);
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setCurrentAutoAimTarget(AUTO_AIM_TARGETS target) {
        this.currentTarget = target;
    }

    private void calibratePeriodic(){
        if (!rightSensorValue && !leftSensorValue){
            if (java.util.Arrays.stream(calibrationPositions).filter(java.util.Objects::nonNull).noneMatch(rotation -> Math.abs(rotation - currentRotationPosition) <= CALIBRATION_THRESHOLD)) {
                if (calibrationPositions[0] == null) {
                    calibrationPositions[0] = currentRotationPosition;
                } else if (calibrationPositions[1] == null) {
                    calibrationPositions[1] = currentRotationPosition;
                    if (calibrationPositions[0] > calibrationPositions[1]){
                        double temp = calibrationPositions[0];
                        calibrationPositions[0] = calibrationPositions[1];
                        calibrationPositions[1] = temp;
                    }
                } else {
                    GreenLogger.log("Shooter.calibrate() has logged more calibration positions than expected");
                }
            }
        }
        if (calibrationPositions[0] != null && calibrationPositions[1] != null && isCalibrating){
            isCalibrating = false;
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

        if(calibrationPositions[0] != null && calibrationPositions[1] != null){
            if (calibrationPositions[0] > rotations || calibrationPositions[1] < rotations){
                GreenLogger.log("Wanted Shooter rotation is out of bounds of the calibrated positions");
            }

            double output = MathUtil.clamp(rotations, calibrationPositions[0], calibrationPositions[1]);

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

}
