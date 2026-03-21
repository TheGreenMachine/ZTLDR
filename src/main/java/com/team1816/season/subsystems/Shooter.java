package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.FieldContainer;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.ShooterTableCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team1816.lib.Singleton.factory;
public class Shooter extends SubsystemBase implements ITestableSubsystem {

    //CLASS
    public static final String NAME = "shooter";

    //STATES
    private ShooterDistanceState wantedDistanceState = ShooterDistanceState.PRESET_CLOSE;

    /**
     * The velocity we want the launch motors to run at (in RPS), assuming {@link
     * #spinUpLaunchMotors} is true.
     */
    private double wantedLaunchVelocityRPS = 0;
    /**
     * If the launch motors should be enabled to spin up.
     */
    private boolean spinUpLaunchMotors = false;
    /**
     * An adjustment value added to all requests to the launch motors (in RPS).
     */
    private double launchVelocityAdjustmentRPS = 0;

    /**
     * The angle we want the incline of the shooter to go to, assuming it is not ducking (in
     * degrees).
     */
    private double wantedInclineAngleDegrees = 0;
    private boolean isInclineDucking = false;
    /**
     * An adjustment value added to all requests to the incline (in degrees).
     */
    private double inclineAngleAdjustmentDegrees = 0;

    /**
     * The angle we want the turret to point at, assuming it is calibrated and not in the dead zone
     * (in degrees).
     */
    private double wantedTurretAngleDegrees = 0;
    private boolean isTurretCalibrated = false;
    private boolean isTurretAimingInDeadZone = false;
    /**
     * The angle to point the turret at when {@link #autoAimTurret} is false (in degrees).
     */
    private double turretFixedAngleDegrees = 0;
    /**
     * If the turret should automatically pick a target and point at it, rather than using the
     * {@link #turretFixedAngleDegrees}.
     */
    private boolean autoAimTurret = true;
    /**
     * If the shooter is automatically aiming in any way that relies on have an accurate pose
     * estimate.
     */
    private boolean isAutoAiming = true;
    /**
     * An adjustment value added to all requests to the turret (in degrees) ONLY USE IN EMERGENCY
     */
    private double turretAngleAdjustmentDegrees = 0;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor inclineMotor = (IMotor) factory.getDevice(NAME, "inclineMotor");
    private final IMotor turretMotor = (IMotor) factory.getDevice(NAME, "turretMotor");
    private final IPhoenix6 candi = (IPhoenix6) factory.getDevice(NAME, "candi");

    private final VelocityVoltage topLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage bottomLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralModeRequest = new NeutralOut();
    private final PositionVoltage inclineMotorPositionRequest = new PositionVoltage(0);
    private final PositionVoltage turretMotorPositionRequest = new PositionVoltage(0);

    //DEVICES
    private final DigitalInput leftTurretSensor = new DigitalInput((int) factory.getConstant(NAME, "leftTurretSensorChannel", 0));
    private final DigitalInput rightTurretSensor = new DigitalInput((int) factory.getConstant(NAME, "rightTurretSensorChannel", 1));

    //HARDWARE RECORDED VALUES
    private boolean leftSensorTriggered = false;
    private boolean rightSensorTriggered = false;
    private boolean sensorValuesHaveBeenSet = false;
    private boolean previousLeftSensorTriggered = false;
    private boolean previousRightSensorTriggered = false;
    private boolean previousSensorValuesKnown = false;

    //CONSTANTS
    private final double MOTOR_ROTATIONS_PER_TURRET_ROTATION;
    private final Translation3d SHOOTER_OFFSET;
    /**
     * The tolerance for the turret rotation to consider it aimed at the target (in degrees).
     */
    private final double TURRET_ROTATION_TOLERANCE_DEGREES;
    /**
     * The tolerance for the incline angle to consider it aimed at the target (in degrees).
     */
    private final double INCLINE_ANGLE_TOLERANCE_DEGREES;
    /**
     * The tolerance for the velocity of the launchers to consider them aimed at the target (in
     * RPS).
     */
    private final double LAUNCHER_VELOCITY_TOLERANCE_RPS;
    /**
     * The maximum angle the incline can go up to for fitting under the trench (in rotations).
     */
    private final double INCLINE_DUCKING_LIMIT_ROTATIONS;
    /**
     * The turret position opposite the dead zone, in turret rotations.
     */
    private final double OPPOSITE_OF_DEAD_ZONE_TURRET_ROTATIONS;
    /**
     * The first (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, in motor rotations.
     */
    private final double FIRST_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
    /**
     * The second (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, in motor rotations.
     */
    private final double SECOND_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
    /**
     * The third (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, in motor rotations.
     */
    private final double THIRD_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
    /**
     * The fourth (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, in motor rotations.
     */
    private final double FOURTH_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
    /**
     * The amount by which to increase or decrease the {@link #launchVelocityAdjustmentRPS} per
     * call to {@link #increaseLaunchVelocityAdjustment()} or {@link
     * #decreaseLaunchVelocityAdjustment()} (in RPS).
     */
    private final double LAUNCH_VELOCITY_ADJUSTMENT_AMOUNT_RPS;
    /**
     * The amount by which to increase or decrease the {@link #inclineAngleAdjustmentDegrees} per
     * call to {@link #increaseInclineAngleAdjustment()} or {@link
     * #decreaseInclineAngleAdjustment()} (in degrees).
     */
    private final double INCLINE_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    /**
     * The amount by which to increase or decrease the {@link #turretAngleAdjustmentDegrees} per
     * call to {@link #increaseTurretAngleAdjustment()} or {@link
     * #decreaseTurretAngleAdjustment()} (in degrees).
     */
    private final double TURRET_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    /**
     * A multiplier on all requests to the {@link #topLaunchMotor} to create backspin on the fuel.
     * Values less than one will cause backspin.
     */
    private final double TOP_LAUNCH_MOTOR_BACKSPIN_MULTIPLIER;

    //CALIBRATION
    /**
     * The offset in motor rotations of the turret motor from the reference frame where robot
     * forward is zero to the positions read by the motor.
     */
    private double turretMotorOffsetRotations;
    private final double FAST_CALIBRATION_SPEED = 0.09;
    private final double SLOW_CALIBRATION_SPEED = 0.04;
    private final DutyCycleOut turretDutyCycleOutRequest = new DutyCycleOut(0);
    private double initialCalibrationStallingTimestamp = -1;
    private final double CALIBRATION_STALL_SECONDS = 1;

    //MECHANISMS
    private final Mechanism2d inclineMech2d = new Mechanism2d(3, 3, new Color8Bit(50, 15, 50));
    private final MechanismRoot2d inclineMechRoot = inclineMech2d.getRoot("Incline Root", 1.5, 0);
    private final MechanismLigament2d inclineMotorML = inclineMechRoot.append(
        new MechanismLigament2d("Incline Angle", 1.5, 0));

    //TARGET TRANSLATION2DS
    // These are all on the blue side, and will be flipped based on alliance. Left and right are
    // from the driver station perspective.
    private final Translation2d HUB_TRANSLATION_2D = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    private final Translation2d LEFT_CORNER_TRANSLATION_2D = new Translation2d(2, 5.07);
    private final Translation2d RIGHT_CORNER_TRANSLATION_2D = new Translation2d(2, 3);
//    private final double ROBOT_STARTING_LINE = Units.inchesToMeters(156.61); True position.
    private final double ROBOT_STARTING_LINE = 4.2684; // Fudged position to shoot into the hub from only partially over the line.

    private final ShooterTableCalculator shooterTableCalculator = new ShooterTableCalculator();

    public Shooter() {
        super();
        // if the turret is ghosted we can say we are calibrated because the motors will not move
        if(turretMotor.isGhost()) isTurretCalibrated = true;
        MOTOR_ROTATIONS_PER_TURRET_ROTATION = factory.getConstant(NAME, "motorRotationsPerTurretRotation", 1);
        SHOOTER_OFFSET = new Translation3d(
            factory.getConstant(NAME, "shooterOffsetXMeters",0),
            factory.getConstant(NAME, "shooterOffsetYMeters",0),
            factory.getConstant(NAME, "shooterOffsetZMeters",0)
        );

        // Find the turret positions of the four spots that we would see beam break values change,
        // in motor rotations relative robot forward.
        double closeDistanceBetweenBeamBreaks = factory.getConstant(NAME, "closeDistanceBetweenBeamBreaks", 0);
        double farDistanceBetweenBeamBreaks = factory.getConstant(NAME, "farDistanceBetweenBeamBreaks", 0);
        double secondLowestBeamBreakToZero = factory.getConstant(NAME, "secondLowestBeamBreakToZero", 0);
        FIRST_BEAM_BREAK_POSITION_MOTOR_ROTATIONS = -secondLowestBeamBreakToZero - closeDistanceBetweenBeamBreaks;
        SECOND_BEAM_BREAK_POSITION_MOTOR_ROTATIONS = -secondLowestBeamBreakToZero;
        THIRD_BEAM_BREAK_POSITION_MOTOR_ROTATIONS = -secondLowestBeamBreakToZero + farDistanceBetweenBeamBreaks;
        FOURTH_BEAM_BREAK_POSITION_MOTOR_ROTATIONS = -secondLowestBeamBreakToZero + farDistanceBetweenBeamBreaks + closeDistanceBetweenBeamBreaks;
        // Get the position opposite of the dead zone to use as the center of our wrapped range.
        double oppositeOfDeadZoneMotorRotations = (
            SECOND_BEAM_BREAK_POSITION_MOTOR_ROTATIONS + THIRD_BEAM_BREAK_POSITION_MOTOR_ROTATIONS
        ) / 2;
        OPPOSITE_OF_DEAD_ZONE_TURRET_ROTATIONS = oppositeOfDeadZoneMotorRotations / MOTOR_ROTATIONS_PER_TURRET_ROTATION; //TODO address possible error w/ motorrotationsperturretrotation

        TURRET_ROTATION_TOLERANCE_DEGREES = factory.getConstant(NAME, "turretRotationToleranceDegrees", 0);
        INCLINE_ANGLE_TOLERANCE_DEGREES = factory.getConstant(NAME, "inclineAngleToleranceDegrees", 0);
        LAUNCHER_VELOCITY_TOLERANCE_RPS = factory.getConstant(NAME, "launcherVelocityToleranceRPS", 0);

        INCLINE_DUCKING_LIMIT_ROTATIONS = factory.getConstant(NAME, "inclineDuckingLimitRotations", 0);

        LAUNCH_VELOCITY_ADJUSTMENT_AMOUNT_RPS = factory.getConstant(NAME, "launchVelocityAdjustmentAmountRPS", 0);
        INCLINE_ANGLE_ADJUSTMENT_AMOUNT_DEGREES = factory.getConstant(NAME, "inclineAngleAdjustmentAmountDegrees", 0);
        TURRET_ANGLE_ADJUSTMENT_AMOUNT_DEGREES = factory.getConstant(NAME, "turretAngleAdjustmentAmountDegrees", 0);

        TOP_LAUNCH_MOTOR_BACKSPIN_MULTIPLIER = factory.getConstant(NAME, "topLaunchMotorBackspinMultiplier", 1);

        GreenLogger.periodicLog(NAME + "/Wanted Distance State", () -> wantedDistanceState);
        GreenLogger.periodicLog(NAME + "/Aimed", this::isAimed);
        GreenLogger.periodicLog(NAME + "/Is Auto Aiming", () -> isAutoAiming);

        // The current launch velocities (in RPS) are already logged by the motor, so we don't need to log them here.
        GreenLogger.periodicLog(NAME + "/launchMotors/Wanted Velocity RPS", () -> wantedLaunchVelocityRPS);
        GreenLogger.periodicLog(NAME + "/launchMotors/Aimed", this::areLaunchMotorsAimed);
        GreenLogger.periodicLog(NAME + "/launchMotors/Spinning Up", () -> spinUpLaunchMotors);
        GreenLogger.periodicLog(NAME + "/launchMotors/Velocity Adjustment RPS", () -> launchVelocityAdjustmentRPS);

        // Because this first one is a Mechanism2d, it will be under the SmartDashboard section of the NetworkTables.
        GreenLogger.periodicLog("Shooter Incline", () -> inclineMech2d);
        GreenLogger.periodicLog(NAME + "/incline/Current Angle Degrees", this::getCurrentInclineAngleDegrees);
        GreenLogger.periodicLog(NAME + "/incline/Wanted Angle Degrees", () -> wantedInclineAngleDegrees);
        GreenLogger.periodicLog(NAME + "/incline/Aimed", this::isInclineAimed);
        GreenLogger.periodicLog(NAME + "/incline/Ducking", () -> isInclineDucking);
        GreenLogger.periodicLog(NAME + "/incline/Angle Adjustment Degrees", () -> inclineAngleAdjustmentDegrees);

        GreenLogger.periodicLog(NAME + "/turret/Field Pose", this::getCurrentTurretPose2d, Pose2d.struct);
        GreenLogger.periodicLog(NAME + "/turret/Current Robot Relative Angle Degrees", () -> getCurrentRobotRelativeTurretRotation2d().getDegrees());
        GreenLogger.periodicLog(NAME + "/turret/Wanted Angle Degrees", () -> wantedTurretAngleDegrees);
        GreenLogger.periodicLog(NAME + "/turret/Aimed", this::isTurretAimed);
        GreenLogger.periodicLog(NAME + "/turret/Calibrated", () -> isTurretCalibrated);
        GreenLogger.periodicLog(NAME + "/turret/Aiming in Dead Zone", () -> isTurretAimingInDeadZone);
        GreenLogger.periodicLog(NAME + "/turret/Left Sensor Triggered", () -> leftSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Right Sensor Triggered", () -> rightSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Motor Offset Rotations", () -> turretMotorOffsetRotations);
        GreenLogger.periodicLog(NAME + "/turret/Fixed Angle Degrees", () -> turretFixedAngleDegrees);
        GreenLogger.periodicLog(NAME + "/turret/Auto Aiming Turret", () -> autoAimTurret);
        GreenLogger.periodicLog(NAME + "/turret/Angle Adjustment Degrees", () -> turretAngleAdjustmentDegrees);
    }

    @Override
    public void periodic() {
        readFromHardware();

        if (turretMotor.hasDeviceCrashed()) {
            isTurretCalibrated = false;
        }

        if (!isTurretCalibrated) {
            calibrateTurretMotor();
        }

        applyState();
    }

    @Override
    public void readFromHardware() {
        // Beam Break Sensor Reading
        if (sensorValuesHaveBeenSet) {
            // If this isn't the first loop, set the previous sensor triggered values to what the
            // sensor triggered values were in the last loop.
            previousLeftSensorTriggered = leftSensorTriggered;
            previousRightSensorTriggered = rightSensorTriggered;
            // Now we have valid values for the previous sensor triggered values.
            previousSensorValuesKnown = true;
        }
        // Set the sensor triggered values from the hardware. get() returning false means the
        // sensor is triggered.
        leftSensorTriggered = !leftTurretSensor.get();
        rightSensorTriggered = !rightTurretSensor.get();
        // Now the sensor triggered values have been set at least once.
        sensorValuesHaveBeenSet = true;

        FieldContainer.field.getObject("Turret").setPose(getCurrentTurretPose2d());

        inclineMotorML.setAngle(getCurrentInclineAngleDegrees());
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private void applyState() {
        Translation2d target = Translation2d.kZero;
        if (autoAimTurret || wantedDistanceState == ShooterDistanceState.AUTOMATIC) {
            isAutoAiming = true;
            target = getTargetTranslation2d();
        }
        else {
            isAutoAiming = false;
        }

        if (autoAimTurret) {
            aimTurretAtTarget(target);
        }
        else {
            setTurretAngle(turretFixedAngleDegrees);
        }

        switch (wantedDistanceState) {
            case IDLE, PRESET_CLOSE, PRESET_MIDDLE, PRESET_FAR, PRESET_AUTO_THING -> {
                setInclineAngle(wantedDistanceState.getInclineAngleDegrees());
                setLaunchVelocities(wantedDistanceState.getLaunchVelocityRPS());
            }
            case AUTOMATIC -> {
                if(useVelocityAdjustmentAlgorithm) {
                    useRobotVelocityAdjustment ?
                        aimLaunchersAtTargetVelocityAdjustmentUsingRobotVelocityAdjustment(target, PLACEHOLDER_ANGLE, chassisSpeeds) :
                        aimLaunchersAtTargetVelocityAdjustment(target, PLACEHOLDER_ANGLE);
                }else {
                    aimInclineAndLaunchersAtTargetShootingTable(target);
                }
            }
        }
    }

    public void setWantedDistanceState(ShooterDistanceState state) {
        this.wantedDistanceState = state;
    }

    /**
     * Sets if the incline should duck down to fit under the trench.
     *
     * @param shouldInclineDuck If the incline should duck down to fit under the trench.
     */
    public void setInclineDucking(boolean shouldInclineDuck) {
        isInclineDucking = shouldInclineDuck;
    }

    /**
     * Sets the angle to point the turret at when {@link #autoAimTurret} is false (in degrees).
     *
     * @param wantedAngleDegrees The angle to point the turret at when {@link #autoAimTurret} is
     *                           false (in degrees).
     */
    public void setTurretFixedAngle(double wantedAngleDegrees) {
        turretFixedAngleDegrees = wantedAngleDegrees;
    }

    /**
     * Sets if the turret should automatically point at either the hub or the corners. If false,
     * the turret will point at the {@link #turretFixedAngleDegrees} instead.
     *
     * @param shouldAutoAimTurret If the turret should aim automatically.
     */
    public void setAutoAimTurret(boolean shouldAutoAimTurret) {
        autoAimTurret = shouldAutoAimTurret;
    }

    /**
     * Sets if the launch motors should be spun up to the {@link #wantedLaunchVelocityRPS}. Pass in
     * false to effectively disable the launch motors.
     *
     * @param shouldSpinUpLaunchMotors If the launch motors should be allowed to spin up.
     */
    public void setSpinUpLaunchMotors(boolean shouldSpinUpLaunchMotors) {
        spinUpLaunchMotors = shouldSpinUpLaunchMotors;
    }

    /**
     * Increases the adjustment value to all requests to the launch motors.
     */
    public void increaseLaunchVelocityAdjustment() {
        launchVelocityAdjustmentRPS += LAUNCH_VELOCITY_ADJUSTMENT_AMOUNT_RPS;
    }

    /**
     * Decreases the adjustment value to all requests to the launch motors.
     */
    public void decreaseLaunchVelocityAdjustment() {
        launchVelocityAdjustmentRPS -= LAUNCH_VELOCITY_ADJUSTMENT_AMOUNT_RPS;
    }

    /**
     * Increases the adjustment value to all requests to the incline.
     */
    public void increaseInclineAngleAdjustment() {
        inclineAngleAdjustmentDegrees += INCLINE_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    }

    /**
     * Decreases the adjustment value to all requests to the incline.
     */
    public void decreaseInclineAngleAdjustment() {
        inclineAngleAdjustmentDegrees -= INCLINE_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    }

    /**
     * Increases the adjustment value to all requests to the turret.
     */
    public void increaseTurretAngleAdjustment() {
        turretAngleAdjustmentDegrees += TURRET_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    }

    /**
     * Decreases the adjustment value to all requests to the turret.
     */
    public void decreaseTurretAngleAdjustment() {
        turretAngleAdjustmentDegrees -= TURRET_ANGLE_ADJUSTMENT_AMOUNT_DEGREES;
    }

    /**
     * Sets the turret back into calibration mode.
     */
    public void recalibrateTurret() {
        isTurretCalibrated = false;
    }

    /**
     * Gets if the incline is ducked low enough to go under the trench.
     *
     * @return If the incline is ducked low enough to go under the trench.
     */
    public boolean isInclineDucked() {
        return getCurrentInclineAngleDegrees() <
            Units.rotationsToDegrees(INCLINE_DUCKING_LIMIT_ROTATIONS)
                + INCLINE_ANGLE_TOLERANCE_DEGREES;
    }

    /**
     * Gets the {@link Translation2d} of the target that we should aim at, based on the location of
     * the robot on the field. Specifically, determines if we should aim at the hub or the corner
     * of the alliance zone, determines which corner to aim at if aiming at the corner, and gets
     * the correct {@link Translation2d} of this target based on the alliance.
     *
     * @return The {@link Translation2d} of the target we should aim at.
     */
    private Translation2d getTargetTranslation2d() {
        Pose2d robotPose = BaseRobotState.robotPose;
        double robotXMeters = robotPose.getX();
        double robotYMeters = robotPose.getY();
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        boolean isInAllianceZone = isBlueAlliance
            ? robotXMeters < ROBOT_STARTING_LINE
            : robotXMeters > FlippingUtil.fieldSizeX - ROBOT_STARTING_LINE;
        if (isInAllianceZone) {
            // Aim at hub
            return flipTranslation2dBasedOnAlliance(HUB_TRANSLATION_2D);
        }
        else {
            // Aim at corner on alliance side. Determine left or right based on which half of the
            // field we are on.
            // Left and right are from driver station perspective, so if we are blue alliance, we
            // are on the left if y is greater than the middle, and if we are red alliance, we are
            // on the left if y is less than the middle.
            if (isBlueAlliance == robotYMeters > (FlippingUtil.fieldSizeY / 2)) {
                return flipTranslation2dBasedOnAlliance(LEFT_CORNER_TRANSLATION_2D);
            }
            else {
                return flipTranslation2dBasedOnAlliance(RIGHT_CORNER_TRANSLATION_2D);
            }
        }
    }

    /**
     * Flips the passed in {@link Translation2d} based on the alliance.
     *
     * @param blueTranslation2d The {@link Translation2d} to flip. This should be the {@link
     * Translation2d} for the blue alliance side.
     * @return The original {@link Translation2d} if the alliance is blue, or the flipped {@link
     * Translation2d} if the alliance is red.
     */
    private Translation2d flipTranslation2dBasedOnAlliance(Translation2d blueTranslation2d) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
            ? blueTranslation2d
            : FlippingUtil.flipFieldPosition(blueTranslation2d);
    }

    /**
     * Automatically points the turret at the passed in target.
     *
     * @param targetTranslation2d The {@link Translation2d} of the target to aim at.
     */
    private void aimTurretAtTarget(Translation2d targetTranslation2d) {
        Translation2d shooterTranslation2d = getCurrentTurretPose2d().getTranslation();
        Translation2d shooterToTargetTranslation2d = targetTranslation2d.minus(shooterTranslation2d);
        Rotation2d fieldRelativeRotation2dToTarget = shooterToTargetTranslation2d.getAngle();
        Rotation2d robotRotation2d = BaseRobotState.robotPose.getRotation();
        Rotation2d robotRelativeRotation2dToTarget = fieldRelativeRotation2dToTarget.minus(robotRotation2d);
        double robotRelativeDegreesToTarget = robotRelativeRotation2dToTarget.getDegrees();
        setTurretAngle(robotRelativeDegreesToTarget);
    }

    /**
     * Automatically points the incline and spins up the launchers to shoot at the passed in target
     * at hub height using the shooter lookup table.
     *
     * @param targetTranslation2d The {@link Translation2d} of the target to aim at.
     */
    private void aimInclineAndLaunchersAtTargetShootingTable(Translation2d targetTranslation2d) {
        Translation2d shooterTranslation2d = getCurrentTurretPose2d().getTranslation();
        double distanceToTargetMeters = shooterTranslation2d.getDistance(targetTranslation2d);
        double distanceToTargetInches = Units.metersToInches(distanceToTargetMeters);
        ShooterTableCalculator.ShooterDistanceSetting shooterDistanceSetting = shooterTableCalculator
            .getShooterDistanceSetting(distanceToTargetInches);
        double inclineAngleRotations = shooterDistanceSetting.inclineAngleRotations();
        double inclineAngleDegrees = Units.rotationsToDegrees(inclineAngleRotations);
        double launchVelocityRPS = shooterDistanceSetting.launchVelocityRPS();
        setInclineAngle(inclineAngleDegrees);
        setLaunchVelocities(launchVelocityRPS);
    }

    public boolean useVelocityAdjustmentAlgorithm = false;
    public boolean useRobotVelocityAdjustment = false;

    private void aimLaunchersAtTargetVelocityAdjustment(Translation2d targetTranslation2d, double inclineAngleDegrees) {
        Translation2d shooterTranslation2d = getCurrentTurretPose2d().getTranslation();
        double distanceToTargetMeters = shooterTranslation2d.getDistance(targetTranslation2d);
        double wantedVelocityMPS = ballisticEquationForVelocity(distanceToTargetMeters, inclineAngleDegrees);
        double launchVelocityRPS = wantedVelocityMPS * (1_METER/CIRCUMFERENCE_METER);
        setLaunchVelocities(launchVelocityRPS);
    }

    private void aimLaunchersAtTargetVelocityAdjustmentUsingRobotVelocityAdjustment(Translation2d targetTranslation2d, double inclineAngleDegrees, ChassisSpeeds chassisSpeeds) {
        Translation2d shooterTranslation2d = getCurrentTurretPose2d().getTranslation();
        double distanceToTargetMeters = shooterTranslation2d.getDistance(targetTranslation2d);
        double wantedVelocityMPS = ballisticEquationForVelocity(distanceToTargetMeters, inclineAngleDegrees);
        double launchVelocityRPS = wantedVelocityMPS * (1_METER/CIRCUMFERENCE_METER);
        double shotPathTime = distanceToTargetMeters / (wantedVelocityMPS * Math.sin(inclineAngleDegrees));
        Translation2d adjustedShooterTranslation2d = shooterTranslation2d.plus(new Translation2d(chassisSpeeds.vxMetersPerSecond*shotPathTime, chassisSpeeds.vyMetersPerSecond*shotPathTime));
        double adjustedDistanceToTargetMeters = shooterTranslation2d.getDistance(adjustedShooterTranslation2d);
        double adjustedWantedVelocityMPS = ballisticEquationForVelocity(adjustedDistanceToTargetMeters, inclineAngleDegrees);
        double adjustedLaunchVelocityRPS = adjustedWantedVelocityMPS * (1_METER/CIRCUMFERENCE_METER);
        setLaunchVelocities(adjustedLaunchVelocityRPS);
    }

    private double ballisticEquationForVelocity(double distanceMeters, double angleDegrees){
        return Math.sqrt(9.81*distanceMeters*distanceMeters/(2*Math.pow(Math.cos(distanceMeters*Math.tan(angleDegrees)-(1.83-SHOOTER_OFFSET.getZ())), 2)));
    }

    /**
     * Determines the offset of the {@link #turretMotor} based on where the beam break
     * sensors change triggered values.
     */
    private void calibrateTurretMotor() {
        // We only want to run the motors to automatically calibrate if the robot is enabled, but the
        // detection process for beam breaks changing will be the same
        if (DriverStation.isEnabled()) {
            // Set the initial stalling timestamp if it hasn't been set yet
            if (initialCalibrationStallingTimestamp == -1 && leftSensorTriggered && rightSensorTriggered) {
                initialCalibrationStallingTimestamp = Timer.getFPGATimestamp();
            }

            // If the timestamp has been initialized and the time elapsed has reached the calibration stalling seconds...
            if (initialCalibrationStallingTimestamp != -1 && Timer.getFPGATimestamp() - initialCalibrationStallingTimestamp >= CALIBRATION_STALL_SECONDS) {
                // Move clockwise slowly
                turretMotor.setControl(turretDutyCycleOutRequest.withOutput(-SLOW_CALIBRATION_SPEED));
            }
            // Otherwise, if the right beam break is tripped...
            else if (rightSensorTriggered) {
                // Move counterclockwise slowly
                turretMotor.setControl(turretDutyCycleOutRequest.withOutput(SLOW_CALIBRATION_SPEED));
            }
            // Otherwise (so if right sensor isn't tripped, and either the initial calibration timestamp hasn't been initialized or it hasn't reached the stalling time)...
            else {
                // Move counterclockwise quickly
                turretMotor.setControl(turretDutyCycleOutRequest.withOutput(FAST_CALIBRATION_SPEED));
            }
        }
        // Reset the initial calibration timestamp in case the robot is ever re-enabled
        else {
            initialCalibrationStallingTimestamp = -1;
        }
        // While there are only two beam break sensors, we can see them on both sides of the
        // turret's range of motion because of the width of the part that blocks them. This means
        // we have four distinct locations we can measure based on the beam breaks. We know we are
        // directly at one of the beam break positions when the beam break becomes triggered or
        // untriggered.
        if (previousSensorValuesKnown) { // Need to know the previous sensor values to check for changes.
            if (leftSensorTriggered != previousLeftSensorTriggered) { // Change in left sensor triggered value.
                if (rightSensorTriggered) {
                    // Must be at the first of the four positions where sensor values change.
                    finishCalibration(FIRST_BEAM_BREAK_POSITION_MOTOR_ROTATIONS);
                }
                else {
                    // Must be at the third of the four positions where sensor values change.
                    finishCalibration(THIRD_BEAM_BREAK_POSITION_MOTOR_ROTATIONS);
                }
            }
            else if (rightSensorTriggered != previousRightSensorTriggered) { // Change in the right sensor triggered value.
                if (leftSensorTriggered) {
                    // Must be at the fourth of the four positions where sensor values change.
                    finishCalibration(FOURTH_BEAM_BREAK_POSITION_MOTOR_ROTATIONS);
                }
                else {
                    // Must be at the second of the four positions where sensor values change.
                    finishCalibration(SECOND_BEAM_BREAK_POSITION_MOTOR_ROTATIONS);
                }
            }
        }
    }

    private void finishCalibration(double beamBreakPositionMotorRotations) {
        turretMotorOffsetRotations = turretMotor.getMotorPosition() - beamBreakPositionMotorRotations;
        turretMotor.setControl(turretDutyCycleOutRequest.withOutput(0));
        isTurretCalibrated = true;
    }

    /**
     * Sends a velocity request to the {@link #topLaunchMotor} and {@link #bottomLaunchMotor} based
     * on the passed in velocity in RPS.
     *
     * @param wantedVelocityRPS The desired velocity of the launch motors (in RPS).
     */
    private void setLaunchVelocities(double wantedVelocityRPS) {
        wantedLaunchVelocityRPS = wantedVelocityRPS + launchVelocityAdjustmentRPS;
        if (spinUpLaunchMotors) {
            topLaunchMotor.setControl(topLaunchMotorVelocityRequest.withVelocity(
                wantedLaunchVelocityRPS * TOP_LAUNCH_MOTOR_BACKSPIN_MULTIPLIER
            ));
            bottomLaunchMotor.setControl(bottomLaunchMotorVelocityRequest.withVelocity(wantedLaunchVelocityRPS));
        }
        else {
            topLaunchMotor.setControl(neutralModeRequest);
            bottomLaunchMotor.setControl(neutralModeRequest);
        }
    }

    /**
     * Gets if the launch motors are spun up to the desired velocity and ready to shoot.
     *
     * @return If the launch motors are aimed.
     */
    private boolean areLaunchMotorsAimed() {
        return MathUtil.isNear(
            wantedLaunchVelocityRPS,
            topLaunchMotor.getMotorVelocity(),
            LAUNCHER_VELOCITY_TOLERANCE_RPS
        )
            && MathUtil.isNear(
                wantedLaunchVelocityRPS,
                bottomLaunchMotor.getMotorVelocity(),
                LAUNCHER_VELOCITY_TOLERANCE_RPS
            );
    }

    /**
     * Sends a position request to the {@link #inclineMotor} based on the passed in angle in
     * degrees. This method will duck the incline under the trench if {@link #isInclineDucking} is
     * true.
     *
     * @param wantedAngleDegrees The desired angle of the incline (in degrees).
     */
    private void setInclineAngle(double wantedAngleDegrees) {
        wantedInclineAngleDegrees = wantedAngleDegrees + inclineAngleAdjustmentDegrees;
        double rotations = Units.degreesToRotations(wantedInclineAngleDegrees);
        if (isInclineDucking) {
            // If we are trying to duck under the trench, restrict the angle of the incline to be
            // below the limit.
            rotations = Math.min(rotations, INCLINE_DUCKING_LIMIT_ROTATIONS);
        }
        inclineMotor.setControl(inclineMotorPositionRequest.withPosition(rotations));
    }

    /**
     * Gets the incline's current angle (in degrees).
     *
     * @return The current angle of the incline (in degrees).
     */
    private double getCurrentInclineAngleDegrees() {
        double rotations = inclineMotor.getMotorPosition();
        return Units.rotationsToDegrees(rotations);
    }

    /**
     * Gets if the incline is aimed at the desired angle and ready to shoot.
     *
     * @return If the incline is aimed.
     */
    private boolean isInclineAimed() {
        return MathUtil.isNear(
            wantedInclineAngleDegrees,
            getCurrentInclineAngleDegrees(),
            INCLINE_ANGLE_TOLERANCE_DEGREES
        );
    }

    /**
     * Sends a position request to the {@link #turretMotor} based on the passed in robot-relative
     * angle in counterclockwise positive degrees. This method will only move the turret is it is
     * calibrated and will avoid the dead zone.
     *
     * @param wantedAngleDegrees The desired robot-relative angle to point the turret at, in
     *                           degrees counterclockwise from forward.
     */
    private void setTurretAngle(double wantedAngleDegrees) {
        wantedTurretAngleDegrees = wantedAngleDegrees + turretAngleAdjustmentDegrees;
        if (isTurretCalibrated) {
            double wantedTurretRotations = Units.degreesToRotations(wantedTurretAngleDegrees);

            double wrappedWantedTurretRotations = MathUtil.inputModulus(
                wantedTurretRotations,
                OPPOSITE_OF_DEAD_ZONE_TURRET_ROTATIONS - 0.5,
                OPPOSITE_OF_DEAD_ZONE_TURRET_ROTATIONS + 0.5
            );
            double wantedMotorRotations = wrappedWantedTurretRotations * MOTOR_ROTATIONS_PER_TURRET_ROTATION;

            // Clamp the position between the first and fourth beam break positions.
            double lowerLimitMotorRotations = FIRST_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
            double upperLimitMotorRotations = FOURTH_BEAM_BREAK_POSITION_MOTOR_ROTATIONS;
            isTurretAimingInDeadZone = wantedMotorRotations < lowerLimitMotorRotations || wantedMotorRotations > upperLimitMotorRotations;
            double clampedMotorRotations = MathUtil.clamp(
                wantedMotorRotations,
                lowerLimitMotorRotations,
                upperLimitMotorRotations
            );

            turretMotor.setControl(turretMotorPositionRequest.withPosition(
                clampedMotorRotations + turretMotorOffsetRotations
            ));
        }
    }

    /**
     * Gets the {@link Pose2d} representing the turret's current pose on the field.
     *
     * @return The current field-relative {@link Pose2d} of the turret.
     */
    private Pose2d getCurrentTurretPose2d() {
        Translation2d robotToTurretTranslation2d = SHOOTER_OFFSET.toTranslation2d();
        Rotation2d robotToTurretRotation2d = getCurrentRobotRelativeTurretRotation2d();

        Transform2d robotToTurretTransform2d = new Transform2d(
            robotToTurretTranslation2d,
            robotToTurretRotation2d
        );

        Pose2d robotPose2d = BaseRobotState.robotPose;

        return robotPose2d.transformBy(robotToTurretTransform2d);
    }

    /**
     * Gets the {@link Rotation2d} representing the turret's current robot-relative rotation.
     *
     * @return The current robot-relative {@link Rotation2d} of the turret.
     */
    private Rotation2d getCurrentRobotRelativeTurretRotation2d() {
        double motorRotations = turretMotor.getMotorPosition();
        double offsetMotorRotations = motorRotations - turretMotorOffsetRotations;
        double turretRotations = offsetMotorRotations / MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        return Rotation2d.fromRotations(turretRotations);
    }

    /**
     * Gets if the turret is aimed at the desired angle and ready to shoot.
     *
     * @return If the turret is aimed.
     */
    private boolean isTurretAimed() {
        return isTurretCalibrated
            && MathUtil.isNear(
                wantedTurretAngleDegrees,
                getCurrentRobotRelativeTurretRotation2d().getDegrees(),
                TURRET_ROTATION_TOLERANCE_DEGREES
            );
    }

    /**
     * Gets if the whole shooter (turret and incline) is aimed at the target and ready to shoot. We
     * are assuming the launch motor spinup time is negligible, so we are not currently checking
     * them.
     *
     * @return If the shooter is aimed.
     */
    public boolean isAimed() {
        return isInclineAimed()
            && isTurretAimed()
            // If we are auto trying to auto aim but don't actually know where we are, we are
            // probably not aimed correctly.
            && !(isAutoAiming && !BaseRobotState.hasAccuratePoseEstimate);
    }

    public enum ShooterDistanceState {
        PRESET_CLOSE(
            Units.rotationsToDegrees(factory.getConstant(NAME,"distanceOneInclineAngleRotations",0)),
            factory.getConstant(NAME,"distanceOneLaunchVelocityRPS",0)
        ),
        PRESET_MIDDLE(
            Units.rotationsToDegrees(factory.getConstant(NAME,"distanceTwoInclineAngleRotations",0)),
            factory.getConstant(NAME,"distanceTwoLaunchVelocityRPS",0)
        ),
        PRESET_FAR(
            Units.rotationsToDegrees(factory.getConstant(NAME,"distanceThreeInclineAngleRotations",0)),
            factory.getConstant(NAME,"distanceThreeLaunchVelocityRPS",0)
        ),
        PRESET_AUTO_THING(
            Units.rotationsToDegrees(factory.getConstant(NAME,"distanceThreeInclineAngleRotations",0)),
            factory.getConstant(NAME,"distanceAutoThingLaunchVelocityRPS",0)
        ),
        AUTOMATIC(-1, -1),
        IDLE(0, 0);

        private final double inclineAngleDegrees, launchVelocityRPS;

        ShooterDistanceState(double inclineAngleDegrees, double launchVelocityRPS) {
            this.inclineAngleDegrees = inclineAngleDegrees;
            this.launchVelocityRPS = launchVelocityRPS;
        }

        double getInclineAngleDegrees() {
            return inclineAngleDegrees;
        }

        double getLaunchVelocityRPS() {
            return launchVelocityRPS;
        }
    }
}
