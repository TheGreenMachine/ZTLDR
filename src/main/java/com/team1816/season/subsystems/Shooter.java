package com.team1816.season.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    private ShooterState wantedState = ShooterState.PRESET_CLOSE;

    /**
     * The velocity we want the launch motors to run at (in RPS).
     */
    private double wantedLaunchVelocityRPS = 0;

    /**
     * The angle we want the incline of the shooter to go to, assuming it is not ducking (in
     * degrees).
     */
    private double wantedInclineAngleDegrees = 0;
    private boolean isInclineDucking = false;

    /**
     * The angle we want the turret to point at, assuming it is calibrated and not in the dead zone
     * (in degrees).
     */
    private double wantedTurretAngleDegrees = 0;
    private boolean isTurretCalibrated = false;
    private boolean isTurretAimingInDeadZone = false;
    /**
     * The angle to point the turret at when using one of the distance presets (in degrees).
     */
    private double turretPresetAngleDegrees = 0;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor inclineMotor = (IMotor) factory.getDevice(NAME, "inclineMotor");
    private final IMotor turretMotor = (IMotor) factory.getDevice(NAME, "turretMotor");
    private final IPhoenix6 candi = (IPhoenix6) factory.getDevice(NAME, "candi");

    private final VelocityVoltage topLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage bottomLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage inclineMotorPositionRequest = new PositionVoltage(0);
    private final PositionVoltage turretMotorPositionRequest = new PositionVoltage(0);

    //AUTO AIM
    private AUTO_AIM_TARGETS currentTarget = AUTO_AIM_TARGETS.RED_HUB;
    // TODO: get the launcher position from the vision or whatever
    private Translation3d launcherTranslation;

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
    private final double HALF_FIELD_WIDTH = FlippingUtil.fieldSizeY/2;
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
     * The maximum angle the incline can go up to for fitting under the trench (in degrees).
     */
    private final double INCLINE_DUCKING_LIMIT_DEGREES;

    //CALIBRATION
    private final double CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS;
    private final double FAR_DISTANCE_BETWEEN_BEAM_BREAKS;
    private final double SECOND_LOWEST_BEAM_BREAK_TO_ZERO;
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

    public Shooter() {
        super();
        // if the turret is ghosted we can say we are calibrated because the motors will not move
        if(turretMotor.isGhost()) isTurretCalibrated = true;
        MOTOR_ROTATIONS_PER_TURRET_ROTATION = factory.getConstant(NAME, "motorRotationsPerTurretRotation", 1);
        SHOOTER_OFFSET = new Translation3d(factory.getConstant(NAME, "initialShooterOffsetX",0), factory.getConstant(NAME, "initialShooterOffsetY",0), factory.getConstant(NAME, "initialShooterOffsetZ",0)); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS = factory.getConstant(NAME, "closeDistanceBetweenBeamBreaks", 0);
        FAR_DISTANCE_BETWEEN_BEAM_BREAKS = factory.getConstant(NAME, "farDistanceBetweenBeamBreaks", 0);
        SECOND_LOWEST_BEAM_BREAK_TO_ZERO = factory.getConstant(NAME, "secondLowestBeamBreakToZero", 0);

        TURRET_ROTATION_TOLERANCE_DEGREES = factory.getConstant(NAME, "turretRotationToleranceDegrees", 0);
        INCLINE_ANGLE_TOLERANCE_DEGREES = factory.getConstant(NAME, "inclineAngleToleranceDegrees", 0);
        LAUNCHER_VELOCITY_TOLERANCE_RPS = factory.getConstant(NAME, "launcherVelocityToleranceRPS", 0);

        INCLINE_DUCKING_LIMIT_DEGREES = factory.getConstant(NAME, "inclineDuckingLimitDegrees", 0);

        launcherTranslation = new Translation3d(0,0,0).plus(SHOOTER_OFFSET);

        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
        GreenLogger.periodicLog(NAME + "/Aimed", this::isAimed);

        // The current launch velocities (in RPS) are already logged by the motor, so we don't need to log them here.
        GreenLogger.periodicLog(NAME + "/launchMotors/Wanted Launch Velocity RPS", () -> wantedLaunchVelocityRPS);
        GreenLogger.periodicLog(NAME + "/launchMotors/Aimed", this::areLaunchMotorsAimed);

        // Because this first one is a Mechanism2d, it will be under the SmartDashboard section of the NetworkTables.
        GreenLogger.periodicLog("Shooter Incline", () -> inclineMech2d);
        GreenLogger.periodicLog(NAME + "/incline/Current Angle Degrees", this::getCurrentInclineAngleDegrees);
        GreenLogger.periodicLog(NAME + "/incline/Wanted Angle Degrees", () -> wantedInclineAngleDegrees);
        GreenLogger.periodicLog(NAME + "/incline/Aimed", this::isInclineAimed);
        GreenLogger.periodicLog(NAME + "/incline/Ducking", () -> isInclineDucking);

        GreenLogger.periodicLog(NAME + "/turret/Field Pose", this::getCurrentTurretPose2d, Pose2d.struct);
        GreenLogger.periodicLog(NAME + "/turret/Current Robot Relative Turret Rotation", this::getCurrentRobotRelativeTurretRotation2d);
        GreenLogger.periodicLog(NAME + "/turret/Wanted Turret Angle Degrees", () -> wantedTurretAngleDegrees);
        GreenLogger.periodicLog(NAME + "/turret/Aimed", this::isTurretAimed);
        GreenLogger.periodicLog(NAME + "/turret/Calibrated", () -> isTurretCalibrated);
        GreenLogger.periodicLog(NAME + "/turret/Aiming in Dead Zone", () -> isTurretAimingInDeadZone);
        GreenLogger.periodicLog(NAME + "/turret/Left Sensor Triggered", () -> leftSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Right Sensor Triggered", () -> rightSensorTriggered);
        GreenLogger.periodicLog(NAME + "/turret/Motor Offset Rotations", () -> turretMotorOffsetRotations);
    }

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

        launcherTranslation = new Translation3d(BaseRobotState.robotPose.getX(), BaseRobotState.robotPose.getY(), 0).plus(SHOOTER_OFFSET);

        FieldContainer.field.getObject("Turret").setPose(getCurrentTurretPose2d());

        inclineMotorML.setAngle(getCurrentInclineAngleDegrees());
    }

    private void applyState() {
        switch (wantedState) {
            case IDLE, PRESET_CLOSE, PRESET_MIDDLE, PRESET_FAR -> {
                setInclineAngle(wantedState.getInclineAngleDegrees());
                setTurretAngle(turretPresetAngleDegrees);
                setLaunchVelocities(wantedState.getLaunchVelocityRPS());
            }
            case AUTOMATIC, AIMING_CORNER, SNOWBLOWING, AIMING_HUB -> {
                // TODO: Implement these. See commented out code below as a starting point.
            }
        }

//        if (wantedState == ShooterState.AUTOMATIC || wantedState == ShooterState.SNOWBLOWING) {
//
//            if (wantedState == ShooterState.AUTOMATIC) {
//                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
//                    currentTarget = AUTO_AIM_TARGETS.RED_HUB;
//                }
//                else {
//                    currentTarget = AUTO_AIM_TARGETS.BLUE_HUB;
//                }
//            }
//            else {
//                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
//                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
//                        currentTarget = AUTO_AIM_TARGETS.RED_RIGHT_CORNER;
//                    }
//                    else {
//                        currentTarget = AUTO_AIM_TARGETS.RED_LEFT_CORNER;
//                    }
//                } else {
//                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
//                        currentTarget = AUTO_AIM_TARGETS.BLUE_LEFT_CORNER;
//                    }
//                    else {
//                        currentTarget = AUTO_AIM_TARGETS.BLUE_RIGHT_CORNER;
//                    }
//                }
//            }
//
//            double distance = launcherTranslation.toTranslation2d().getDistance(currentTarget.position.toTranslation2d());
//
//            ShooterDistanceSetting shooterDistanceSetting = shooterTableCalculator.getShooterDistanceSetting(distance);
//            launchAngle = shooterDistanceSetting.getAngle();
//            launchPower = shooterDistanceSetting.getPower();
//            rotationAngle = Math.tan((launcherTranslation.getY()-currentTarget.position.getY())/(launcherTranslation.getX()-currentTarget.position.getX()));
//        }
//        if(wantedState == ShooterState.AIMING_HUB || wantedState == ShooterState.AIMING_CORNER) {
//            if(wantedState == ShooterState.AIMING_HUB){
//                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
//                    currentTarget = AUTO_AIM_TARGETS.RED_HUB;
//                }
//                else {
//                    currentTarget = AUTO_AIM_TARGETS.BLUE_HUB;
//                }
//            }
//            if(wantedState == ShooterState.AIMING_CORNER){
//                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
//                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
//                        currentTarget = (AUTO_AIM_TARGETS.RED_RIGHT_CORNER);
//                    }
//                    else {
//                        currentTarget = AUTO_AIM_TARGETS.RED_LEFT_CORNER;
//                    }
//                } else {
//                    if (launcherTranslation.getY() < HALF_FIELD_WIDTH) {
//                        currentTarget = AUTO_AIM_TARGETS.BLUE_LEFT_CORNER;
//                    }
//                    else {
//                        currentTarget = AUTO_AIM_TARGETS.BLUE_RIGHT_CORNER;
//                    }
//                }
//            }
//
//            double distance = launcherTranslation.toTranslation2d().getDistance(currentTarget.position.toTranslation2d());
//
//            ShooterDistanceSetting shooterDistanceSetting = shooterTableCalculator.getShooterDistanceSetting(distance);
//            if(shooterDistanceSetting.getAngle() > maxLaunchAngle) {
//                launchAngle = maxLaunchAngle;
//            } else {
//                launchAngle = shooterDistanceSetting.getAngle();
//            }
//            launchPower = shooterDistanceSetting.getPower();
//            rotationAngle = Math.tan((launcherTranslation.getY()-currentTarget.position.getY())/(launcherTranslation.getX()-currentTarget.position.getX()));
//        }
    }

    public void setWantedState(ShooterState state) {
        this.wantedState = state;
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
     * Sets the angle to point the turret at when using one of the distance presets (in degrees).
     *
     * @param wantedAngleDegrees The angle to point the turret at when using one of the distance
     *                           presets (in degrees).
     */
    public void setTurretPresetAngle(double wantedAngleDegrees) {
        turretPresetAngleDegrees = wantedAngleDegrees;
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
                    finishCalibration(-SECOND_LOWEST_BEAM_BREAK_TO_ZERO + -CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS);
                }
                else {
                    // Must be at the third of the four positions where sensor values change.
                    finishCalibration(FAR_DISTANCE_BETWEEN_BEAM_BREAKS - SECOND_LOWEST_BEAM_BREAK_TO_ZERO);
                }
            }
            else if (rightSensorTriggered != previousRightSensorTriggered) { // Change in the right sensor triggered value.
                if (leftSensorTriggered) {
                    // Must be at the fourth of the four positions where sensor values change.
                    finishCalibration(FAR_DISTANCE_BETWEEN_BEAM_BREAKS - SECOND_LOWEST_BEAM_BREAK_TO_ZERO + CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS);
                }
                else {
                    // Must be at the second of the four positions where sensor values change.
                    finishCalibration(-SECOND_LOWEST_BEAM_BREAK_TO_ZERO);
                }
            }
        }
    }

    private void finishCalibration(double beamBreakPositionMotorRotations) {
        turretMotorOffsetRotations = turretMotor.getMotorPosition() - beamBreakPositionMotorRotations;
        turretMotor.setControl(turretDutyCycleOutRequest.withOutput(0));
        isTurretCalibrated = true;
    }

    private void setAutomaticRotationAngle() {
        if(wantedState != ShooterState.AUTOMATIC) return;

        Rotation2d robotCentricWantedAngle =
            currentTarget.getPosition().toTranslation2d()
                .minus(BaseRobotState.robotPose.getTranslation())
                .getAngle()
                .minus(BaseRobotState.robotPose.getRotation());
    }

    /**
     * Sends a velocity request to the {@link #topLaunchMotor} and {@link #bottomLaunchMotor} based
     * on the passed in velocity in RPS.
     *
     * @param wantedVelocityRPS The desired velocity of the launch motors (in RPS).
     */
    private void setLaunchVelocities(double wantedVelocityRPS) {
        wantedLaunchVelocityRPS = wantedVelocityRPS;
        topLaunchMotor.setControl(topLaunchMotorVelocityRequest.withVelocity(wantedLaunchVelocityRPS));
        bottomLaunchMotor.setControl(bottomLaunchMotorVelocityRequest.withVelocity(wantedLaunchVelocityRPS));
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
        wantedInclineAngleDegrees = wantedAngleDegrees;
        double rotations = Units.degreesToRotations(wantedInclineAngleDegrees);
        if (isInclineDucking) {
            // If we are trying to duck under the trench, restrict the angle of the incline to be
            // below the limit.
            rotations = Math.min(rotations, INCLINE_DUCKING_LIMIT_DEGREES);
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
        wantedTurretAngleDegrees = wantedAngleDegrees;
        if (isTurretCalibrated) {
            double wantedTurretRotations = Units.degreesToRotations(wantedTurretAngleDegrees);

            // Get the position opposite of the dead zone to use as the center of our wrapped range.
            double oppositeOfDeadZoneMotorRotations = -SECOND_LOWEST_BEAM_BREAK_TO_ZERO + FAR_DISTANCE_BETWEEN_BEAM_BREAKS/2;
            double oppositeOfDeadZoneTurretRotations = oppositeOfDeadZoneMotorRotations / MOTOR_ROTATIONS_PER_TURRET_ROTATION;

            double wrappedWantedTurretRotations = MathUtil.inputModulus(
                wantedTurretRotations,
                oppositeOfDeadZoneTurretRotations - 0.5,
                oppositeOfDeadZoneTurretRotations + 0.5
            );
            double wantedMotorRotations = wrappedWantedTurretRotations * MOTOR_ROTATIONS_PER_TURRET_ROTATION;

            // Clamp the position between the first and fourth beam break positions.
            double lowerLimitMotorRotations = -SECOND_LOWEST_BEAM_BREAK_TO_ZERO - CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS;
            double upperLimitMotorRotations = -SECOND_LOWEST_BEAM_BREAK_TO_ZERO + FAR_DISTANCE_BETWEEN_BEAM_BREAKS + CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS;
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
        Pose2d robotPose = BaseRobotState.robotPose;

        // Just show the turret in the center of the robot for now.
        Translation2d fieldRelativeTurretTranslation2d = robotPose.getTranslation();

        Rotation2d fieldRelativeRobotRotation2d = robotPose.getRotation();
        Rotation2d robotRelativeTurretRotation2d = getCurrentRobotRelativeTurretRotation2d();
        Rotation2d fieldRelativeTurretRotation2d = fieldRelativeRobotRotation2d.plus(robotRelativeTurretRotation2d);

        return new Pose2d(
            fieldRelativeTurretTranslation2d,
            fieldRelativeTurretRotation2d
        );
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
     * Gets if the whole shooter (turret, incline, and launch motors) is aimed at the target and
     * ready to shoot.
     *
     * @return If the shooter is aimed.
     */
    public boolean isAimed() {
        return areLaunchMotorsAimed() && isInclineAimed() && isTurretAimed();
    }

    public enum ShooterState {
        PRESET_CLOSE(
            factory.getConstant(NAME,"distanceOneInclineAngleDegrees",0),
            factory.getConstant(NAME,"distanceOneLaunchVelocityRPS",0)
        ),
        PRESET_MIDDLE(
            factory.getConstant(NAME,"distanceTwoInclineAngleDegrees",0),
            factory.getConstant(NAME,"distanceTwoLaunchVelocityRPS",0)
        ),
        PRESET_FAR(
            factory.getConstant(NAME,"distanceThreeInclineAngleDegrees",0),
            factory.getConstant(NAME,"distanceThreeLaunchVelocityRPS",0)
        ),
        AUTOMATIC(-1, -1),
        AIMING_HUB(-1, -1),
        SNOWBLOWING(-1, -1),
        AIMING_CORNER(-1, -1),
        IDLE(0, 0);

        private final double inclineAngleDegrees, launchVelocityRPS;

        ShooterState(double inclineAngleDegrees, double launchVelocityRPS) {
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
