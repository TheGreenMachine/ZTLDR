package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.*;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.IPhoenix6;
import com.team1816.lib.hardware.components.motor.IMotor;
import com.team1816.lib.subsystems.ITestableSubsystem;
import com.team1816.lib.util.FieldContainer;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.ShooterCalculator.HenryShooterCalculator;
import com.team1816.lib.util.ShooterCalculator.IShooterCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
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
     * An adjustment value added to all requests to the turret (in degrees).
     */
    private double turretAngleAdjustmentDegrees = 0;

    private boolean useChassisSpeedForHoodAngleAndSpeed = false;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor inclineMotor = (IMotor) factory.getDevice(NAME, "inclineMotor");
    private final IMotor turretMotor = (IMotor) factory.getDevice(NAME, "turretMotor");
    private final IPhoenix6 candi = (IPhoenix6) factory.getDevice(NAME, "candi");
    // put this back in if we want to set the magneticsensor information as this will force the load from yaml to happen
    // private final IPhoenix6 inclineCoder = factory.getDevice(NAME, "inclineCoder");

    private final VelocityVoltage topLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage bottomLaunchMotorVelocityRequest = new VelocityVoltage(0);
    //private final NeutralOut neutralModeRequest = new NeutralOut();
    private final VelocityVoltage neutralModeRequest = new VelocityVoltage(10);

    private final MotionMagicExpoVoltage inclineMotorPositionRequest = new MotionMagicExpoVoltage(0);
    private final MotionMagicExpoVoltage turretMotorPositionRequest = new MotionMagicExpoVoltage(0);

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
     * The turret position opposite the dead zone, (in rotations).
     */
    private final double OPPOSITE_OF_DEAD_ZONE_POSITION_ROTATIONS;
    /**
     * The first (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, (in rotations).
     */
    private final double FIRST_BEAM_BREAK_POSITION_ROTATIONS;
    /**
     * The second (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, (in rotations).
     */
    private final double SECOND_BEAM_BREAK_POSITION_ROTATIONS;
    /**
     * The third (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, (in rotations).
     */
    private final double THIRD_BEAM_BREAK_POSITION_ROTATIONS;
    /**
     * The fourth (in the counterclockwise direction) of the four positions of the turret motor
     * where we would see beam break values change, (in rotations).
     */
    private final double FOURTH_BEAM_BREAK_POSITION_ROTATIONS;
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
    private final Translation3d HUB_TRANSLATION_3D = new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Units.inchesToMeters(72));
    private final Translation3d LEFT_CORNER_TRANSLATION_3D = new Translation3d(2, 5.07, 0);
    private final Translation3d RIGHT_CORNER_TRANSLATION_3D = new Translation3d(2, 3, 0);
//    private final double ROBOT_STARTING_LINE = Units.inchesToMeters(156.61); True position.
    private final double ROBOT_STARTING_LINE = 4.2684; // Fudged position to shoot into the hub from only partially over the line.

    private final IShooterCalculator shooterTableCalculator;

    private Translation3d target = Translation3d.kZero;
    private Pose3d turretPose = Pose3d.kZero;
    private boolean isBlueAlliance = false;

    public Shooter() {
        super();
        // if the turret is ghosted we can say we are calibrated because the motors will not move
        if(turretMotor.isGhost()) isTurretCalibrated = true;
        SHOOTER_OFFSET = new Translation3d(
            factory.getConstant(NAME, "shooterOffsetXMeters",0),
            factory.getConstant(NAME, "shooterOffsetYMeters",0),
            factory.getConstant(NAME, "shooterOffsetZMeters",0)
        );
        // Just change this line to use a new ShooterTableCalculator to switch the calculator type.
        shooterTableCalculator = new HenryShooterCalculator(HUB_TRANSLATION_3D.getZ() - SHOOTER_OFFSET.getZ());

        // Find the turret positions of the four spots that we would see beam break values change.
        double closeDistanceBetweenBeamBreaks = factory.getConstant(NAME, "closeDistanceBetweenBeamBreaks", 0);
        double farDistanceBetweenBeamBreaks = factory.getConstant(NAME, "farDistanceBetweenBeamBreaks", 0);
        double secondLowestBeamBreakToZero = factory.getConstant(NAME, "secondLowestBeamBreakToZero", 0);
        FIRST_BEAM_BREAK_POSITION_ROTATIONS = -secondLowestBeamBreakToZero - closeDistanceBetweenBeamBreaks;
        SECOND_BEAM_BREAK_POSITION_ROTATIONS = -secondLowestBeamBreakToZero;
        THIRD_BEAM_BREAK_POSITION_ROTATIONS = -secondLowestBeamBreakToZero + farDistanceBetweenBeamBreaks;
        FOURTH_BEAM_BREAK_POSITION_ROTATIONS = -secondLowestBeamBreakToZero + farDistanceBetweenBeamBreaks + closeDistanceBetweenBeamBreaks;
        // Get the position opposite of the dead zone to use as the center of our wrapped range.
        OPPOSITE_OF_DEAD_ZONE_POSITION_ROTATIONS = (
            SECOND_BEAM_BREAK_POSITION_ROTATIONS + THIRD_BEAM_BREAK_POSITION_ROTATIONS
        ) / 2;

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

        GreenLogger.periodicLog(NAME + "/turret/Aimed", this::isTurretAimed);
        GreenLogger.periodicLog(NAME + "/turret/Calibrated", () -> isTurretCalibrated);
        GreenLogger.periodicLog(NAME + "/turret/Left Sensor Triggered", () -> leftSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Right Sensor Triggered", () -> rightSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Fixed Angle Degrees", () -> turretFixedAngleDegrees);
        GreenLogger.periodicLog(NAME + "/turret/Auto Aiming Turret", () -> autoAimTurret);
        GreenLogger.periodicLog(NAME + "/turret/Angle Adjustment Degrees", () -> turretAngleAdjustmentDegrees);
        GreenLogger.periodicLog(NAME + "/turret/Is Blue Alliance", () -> isBlueAlliance);
        GreenLogger.periodicLog(NAME + "/turret/calc/Turret Pose", () -> turretPose, Pose3d.struct);
        GreenLogger.periodicLog(NAME + "/turret/calc/Target Translation", () -> target, Translation3d.struct);
        GreenLogger.periodicLog(NAME + "/turret/calc/Robot Pose", () -> BaseRobotState.robotPose, Pose2d.struct);
        GreenLogger.periodicLog(NAME + "/turret/calc/Shooter Offset", () -> SHOOTER_OFFSET, Translation3d.struct);
        GreenLogger.periodicLog(
            NAME + "/turret/calc/Distance to Target",
            // Get the 2d distance between the turret and the target.
            () -> turretPose.getTranslation().toTranslation2d().getDistance(target.toTranslation2d())
        );
        GreenLogger.periodicLog(NAME + "/turret/calc/Wanted Angle Degrees", () -> wantedTurretAngleDegrees);
        GreenLogger.periodicLog(
            NAME + "/turret/calc/Current Angle Degrees",
            () -> getCurrentRobotRelativeTurretRotation2d().getDegrees()
        );
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

        FieldContainer.field.getObject("Turret").setPose(getCurrentTurretPose3d().toPose2d());

        inclineMotorML.setAngle(getCurrentInclineAngleDegrees());
    }

    private void applyState() {
        target = getTargetTranslation3d();
        final double calculatorAngleOfEntryDegrees = 45;

        isAutoAiming = autoAimTurret || wantedDistanceState == ShooterDistanceState.AUTOMATIC;

        if (autoAimTurret) {
            double turretLookAheadTimeSeconds = 0;
            IShooterCalculator.ShooterCalculatorResponse calculatorResponse = shooterTableCalculator.calculate(
                getCurrentTurretPose3d().getTranslation(),
                target,
                calculatorAngleOfEntryDegrees,
                useChassisSpeedForHoodAngleAndSpeed,
                turretLookAheadTimeSeconds
            );
            setTurretAngle(calculatorResponse.turretAngleDegrees());
        }
        else {
            setTurretAngle(turretFixedAngleDegrees);
        }

        switch (wantedDistanceState) {
            case IDLE, PRESET_CLOSE, PRESET_MIDDLE, PRESET_FAR, PRESET_BROKEN_INCLINE_AUTO -> {
                setInclineAngle(wantedDistanceState.getInclineAngleDegrees());
                setLaunchVelocities(wantedDistanceState.getLaunchVelocityRPS());
            }
            case AUTOMATIC -> {
                if (
                    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                        ? BaseRobotState.robotPose.getX() > FlippingUtil.fieldSizeX - ROBOT_STARTING_LINE
                        : BaseRobotState.robotPose.getX() < ROBOT_STARTING_LINE
                ) {
                    setInclineAngle(ShooterDistanceState.PRESET_FAR_SNOWBLOW.getInclineAngleDegrees());
                    setLaunchVelocities(ShooterDistanceState.PRESET_FAR_SNOWBLOW.getLaunchVelocityRPS());
                }
                else {
                    IShooterCalculator.ShooterCalculatorResponse calculatorResponse = shooterTableCalculator.calculate(
                        getCurrentTurretPose3d().getTranslation(),
                        target,
                        calculatorAngleOfEntryDegrees,
                        useChassisSpeedForHoodAngleAndSpeed,
                        0
                    );
                    setInclineAngle(calculatorResponse.inclineAngleDegrees());
                    setLaunchVelocities(calculatorResponse.launchVelocityRPS());
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
     * Gets the {@link Translation3d} of the target that we should aim at, based on the location of
     * the robot on the field. Specifically, determines if we should aim at the hub or the corner
     * of the alliance zone, determines which corner to aim at if aiming at the corner, and gets
     * the correct {@link Translation3d} of this target based on the alliance.
     *
     * @return The {@link Translation3d} of the target we should aim at.
     */
    private Translation3d getTargetTranslation3d() {
        Pose2d robotPose = BaseRobotState.robotPose;
        double robotXMeters = robotPose.getX();
        double robotYMeters = robotPose.getY();
        isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        boolean isInAllianceZone = isBlueAlliance
            ? robotXMeters < ROBOT_STARTING_LINE
            : robotXMeters > FlippingUtil.fieldSizeX - ROBOT_STARTING_LINE;
        if (isInAllianceZone) {
            // Aim at hub
            return flipTranslation3dBasedOnAlliance(HUB_TRANSLATION_3D);
        }
        else {
            // Aim at corner on alliance side. Determine left or right based on which half of the
            // field we are on.
            // Left and right are from driver station perspective, so if we are blue alliance, we
            // are on the left if y is greater than the middle, and if we are red alliance, we are
            // on the left if y is less than the middle.
            if (isBlueAlliance == robotYMeters > (FlippingUtil.fieldSizeY / 2)) {
                return flipTranslation3dBasedOnAlliance(LEFT_CORNER_TRANSLATION_3D);
            }
            else {
                return flipTranslation3dBasedOnAlliance(RIGHT_CORNER_TRANSLATION_3D);
            }
        }
    }

    /**
     * Flips the passed in {@link Translation3d} based on the alliance.
     *
     * @param blueTranslation3d The {@link Translation3d} to flip. This should be the {@link
     * Translation3d} for the blue alliance side.
     * @return The original {@link Translation3d} if the alliance is blue, or the flipped {@link
     * Translation3d} if the alliance is red.
     */
    private Translation3d flipTranslation3dBasedOnAlliance(Translation3d blueTranslation3d) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return blueTranslation3d;
        }
        else {
            // Flip the 2d part with PathPlanner's FlippingUtil.
            Translation2d flippedTranslation2d = FlippingUtil.flipFieldPosition(
                blueTranslation3d.toTranslation2d()
            );
            // Reassemble the Translation3d with the flipped Translation2d and the original height.
            return new Translation3d(
                flippedTranslation2d.getX(),
                flippedTranslation2d.getY(),
                blueTranslation3d.getZ()
            );
        }
    }

    /**
     * Moves the turret until the beam break sensors change triggered values and uses this to zero
     * the {@link #turretMotor}.
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
                    finishCalibration(FIRST_BEAM_BREAK_POSITION_ROTATIONS);
                }
                else {
                    // Must be at the third of the four positions where sensor values change.
                    finishCalibration(THIRD_BEAM_BREAK_POSITION_ROTATIONS);
                }
            }
            else if (rightSensorTriggered != previousRightSensorTriggered) { // Change in the right sensor triggered value.
                if (leftSensorTriggered) {
                    // Must be at the fourth of the four positions where sensor values change.
                    finishCalibration(FOURTH_BEAM_BREAK_POSITION_ROTATIONS);
                }
                else {
                    // Must be at the second of the four positions where sensor values change.
                    finishCalibration(SECOND_BEAM_BREAK_POSITION_ROTATIONS);
                }
            }
        }
    }

    private void finishCalibration(double beamBreakPositionRotations) {
        // Tell the turret motor that it is now at the specific beam break position.
        turretMotor.setPosition(beamBreakPositionRotations);
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
            double wantedRotations = Units.degreesToRotations(wantedTurretAngleDegrees);

            double wrappedWantedRotations = MathUtil.inputModulus(
                wantedRotations,
                OPPOSITE_OF_DEAD_ZONE_POSITION_ROTATIONS - 0.5,
                OPPOSITE_OF_DEAD_ZONE_POSITION_ROTATIONS + 0.5
            );

            turretMotor.setControl(turretMotorPositionRequest.withPosition(wrappedWantedRotations));
        }
    }

    /**
     * Gets the {@link Pose3d} representing the turret's current pose on the field.
     *
     * @return The current field-relative {@link Pose3d} of the turret.
     */
    private Pose3d getCurrentTurretPose3d() {
        Rotation2d robotToTurretRotation2d = getCurrentRobotRelativeTurretRotation2d();

        Transform3d robotToTurretTransform3d = new Transform3d(
            SHOOTER_OFFSET,
            new Rotation3d(robotToTurretRotation2d)
        );

        Pose3d robotPose3d = new Pose3d(BaseRobotState.robotPose);

        turretPose = robotPose3d.transformBy(robotToTurretTransform3d);

        return turretPose;
    }

    /**
     * Gets the {@link Rotation2d} representing the turret's current robot-relative rotation.
     *
     * @return The current robot-relative {@link Rotation2d} of the turret.
     */
    private Rotation2d getCurrentRobotRelativeTurretRotation2d() {
        double turretRotations = turretMotor.getMotorPosition();
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
            && isTurretAimed();
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
        PRESET_BROKEN_INCLINE_AUTO(
            Units.rotationsToDegrees(factory.getConstant(NAME,"distanceThreeInclineAngleRotations",0)),
            factory.getConstant(NAME,"brokenInclineAutoLaunchVelocityRPS",0)
        ),
        PRESET_FAR_SNOWBLOW(
            Units.rotationsToDegrees(factory.getConstant(NAME,"farSnowblowInclineAngleRotations",0)),
            factory.getConstant(NAME,"farSnowblowLaunchVelocityRPS",0)
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
