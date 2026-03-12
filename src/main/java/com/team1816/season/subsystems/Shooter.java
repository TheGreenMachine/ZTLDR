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
import com.team1816.lib.util.ShooterDistanceSetting;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.ShooterTableCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    private SHOOTER_STATE wantedState = SHOOTER_STATE.DISTANCE_ONE;
    private SHOOTER_STATE previousWantedState = SHOOTER_STATE.DISTANCE_ONE;

    //MOTORS
    private final IMotor topLaunchMotor = (IMotor) factory.getDevice(NAME, "topLaunchMotor");
    private final IMotor bottomLaunchMotor = (IMotor) factory.getDevice(NAME, "bottomLaunchMotor");
    private final IMotor launchAngleMotor = (IMotor) factory.getDevice(NAME, "launchAngleMotor");
    private final IMotor rotationAngleMotor = (IMotor) factory.getDevice(NAME, "rotationAngleMotor");
    private final IPhoenix6 candi = (IPhoenix6) factory.getDevice(NAME, "candi");

    private final VelocityVoltage topLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage bottomLaunchMotorVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage launchAnglePositionRequest = new PositionVoltage(0);
    private final PositionVoltage turretPositionRequest = new PositionVoltage(0);

    //AUTO AIM
    private AUTO_AIM_TARGETS currentTarget = AUTO_AIM_TARGETS.RED_HUB;
    // TODO: get the launcher position from the vision or whatever
    private Translation3d launcherTranslation;

    //DEVICES
    private final DigitalInput rotationAngleSensorClockwiseLeft = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseLeft", 0));
    private final DigitalInput rotationAngleSensorClockwiseRight = new DigitalInput((int) factory.getConstant(NAME, "rotationAngleSensorClockwiseRight", 1));

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
    private final double LAUNCH_ANGLE_MOTOR_BOTTOM_LIMIT_ROTATIONS;
    private final double LAUNCH_ANGLE_MOTOR_TOP_LIMIT_ROTATIONS;
    public double maxLaunchAngle = 0; //<-SET MAX ANGLE HERE

    //CALIBRATION
    private final double CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS;
    private final double FAR_DISTANCE_BETWEEN_BEAM_BREAKS;
    private final double SECOND_LOWEST_BEAM_BREAK_TO_ZERO;
    private boolean isCalibrated = false;
    private double rotationAngleMotorOffsetRotations;
    private final double FAST_CALIBRATION_SPEED = 0.08;
    private final double SLOW_CALIBRATION_SPEED = 0.04;
    private final DutyCycleOut turretDutyCycleOutRequest = new DutyCycleOut(0);
    private double initialCalibrationStallingTimestamp = -1;
    private final double CALIBRATION_STALL_SECONDS = 1;

    //MECHANISMS
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
        AIMING_HUB(-1,-1,0),
        SNOWBLOWING(-1, -1, -1),
        AIMING_CORNER(-1,-1,-1),
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
        // if the turret is ghosted we can say we are calibrated because the motors will not move
        if(rotationAngleMotor.isGhost()) isCalibrated = true;
        MOTOR_ROTATIONS_PER_TURRET_ROTATION = factory.getConstant(NAME, "motorRotationsPerTurretRotation", 1);
        SHOOTER_OFFSET = new Translation3d(factory.getConstant(NAME, "initialShooterOffsetX",0), factory.getConstant(NAME, "initialShooterOffsetY",0), factory.getConstant(NAME, "initialShooterOffsetZ",0)); //TODO WHEN PHYSICAL SUBSYSTEM EXISTS, set this.
        CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS = factory.getConstant(NAME, "closeDistanceBetweenBeamBreaks", 0);
        FAR_DISTANCE_BETWEEN_BEAM_BREAKS = factory.getConstant(NAME, "farDistanceBetweenBeamBreaks", 0);
        SECOND_LOWEST_BEAM_BREAK_TO_ZERO = factory.getConstant(NAME, "secondLowestBeamBreakToZero", 0);
        LAUNCH_ANGLE_MOTOR_BOTTOM_LIMIT_ROTATIONS = factory.getConstant(NAME, "launchAngleMotorBottomLimitRotations", 0);
        LAUNCH_ANGLE_MOTOR_TOP_LIMIT_ROTATIONS = factory.getConstant(NAME, "launchAngleMotorTopLimitRotations", 0);

        launcherTranslation = new Translation3d(0,0,0).plus(SHOOTER_OFFSET);

        GreenLogger.periodicLog("Shooter Incline", () -> launchMech);
        GreenLogger.periodicLog(NAME + "/Turret Field Pose", this::getCurrentTurretPose2d, Pose2d.struct);
        GreenLogger.periodicLog(NAME + "/Calibrated", this::isCalibrated);
        GreenLogger.periodicLog(NAME + "/Left Sensor Triggered", () -> leftSensorTriggered);
        GreenLogger.periodicLog(NAME + "/Right Sensor Triggered", () -> rightSensorTriggered);
        GreenLogger.periodicLog(NAME + "/Wanted State", () -> wantedState);
        GreenLogger.periodicLog(NAME + "/Wanted Launch Angle Degrees", () -> wantedState.launchAngle);
        GreenLogger.periodicLog(NAME + "/Wanted Launch Velocity RPS", () -> wantedState.launchVelocity);
        GreenLogger.periodicLog(NAME + "/Wanted Turret Angle Degrees", () -> wantedState.rotationAngle);
        GreenLogger.periodicLog(
            NAME + "/Rotation Angle Motor Offset Rotations", () -> rotationAngleMotorOffsetRotations
        );
    }

    public void periodic() {
        readFromHardware();

        // TODO: Uncomment this once we add automatic calibration.
//        if (rotationAngleMotor.hasDeviceCrashed()) {
//            isCalibrated = false;
//        }

        if (!isCalibrated) {
            calibratePeriodic();
        } else {
            applyState();
        }
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
        leftSensorTriggered = !rotationAngleSensorClockwiseLeft.get();
        rightSensorTriggered = !rotationAngleSensorClockwiseRight.get();
        // Now the sensor triggered values have been set at least once.
        sensorValuesHaveBeenSet = true;

        launcherTranslation = new Translation3d(BaseRobotState.robotPose.getX(), BaseRobotState.robotPose.getY(), 0).plus(SHOOTER_OFFSET);

        FieldContainer.field.getObject("Turret").setPose(getCurrentTurretPose2d());

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
        if(wantedState == SHOOTER_STATE.AIMING_HUB || wantedState == SHOOTER_STATE.AIMING_CORNER) {
            if(wantedState == SHOOTER_STATE.AIMING_HUB){
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                    setCurrentAutoAimTarget(AUTO_AIM_TARGETS.RED_HUB);
                }
                else {
                    setCurrentAutoAimTarget(AUTO_AIM_TARGETS.BLUE_HUB);
                }
            }
            if(wantedState == SHOOTER_STATE.AIMING_CORNER){
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
            if(shooterDistanceSetting.getAngle() > maxLaunchAngle) {
                launchAngle = maxLaunchAngle;
            } else {
                launchAngle = shooterDistanceSetting.getAngle();
            }
            launchPower = shooterDistanceSetting.getPower();
            rotationAngle = Math.tan((launcherTranslation.getY()-currentTarget.position.getY())/(launcherTranslation.getX()-currentTarget.position.getX()));

        }

        setLaunchAngle(launchAngle);
        setRotationAngle(rotationAngle);
        setLaunchMotorVelocities(launchPower);

        previousWantedState = wantedState;
    }

    public void setWantedState(SHOOTER_STATE state) {
        this.wantedState = state;
    }

    public void setCurrentAutoAimTarget(AUTO_AIM_TARGETS target) {
        this.currentTarget = target;
    }

    /**
     * Determines the offset of the {@link #rotationAngleMotor} based on where the beam break
     * sensors change triggered values.
     */
    private void calibratePeriodic() {
        // We only want to run the motors to automatically calibrate if the robot is enabled, but the
        // detection process for beam breaks changing will be the same
        if (DriverStation.isEnabled()) {
            // Set the initial stalling timestamp if it hasn't been set yet
            if (initialCalibrationStallingTimestamp == -1) {
                initialCalibrationStallingTimestamp = Timer.getFPGATimestamp();
            }

            // If the timestamp has been initialized and the time elapsed has reached the calibration stalling seconds...
            if (initialCalibrationStallingTimestamp != -1 && Timer.getFPGATimestamp() - initialCalibrationStallingTimestamp >= CALIBRATION_STALL_SECONDS) {
                // Move clockwise slowly
                rotationAngleMotor.setControl(turretDutyCycleOutRequest.withOutput(-SLOW_CALIBRATION_SPEED));
            }
            // Otherwise, if the right beam break is tripped...
            else if (!rotationAngleSensorClockwiseRight.get()) {
                // Move counterclockwise slowly
                rotationAngleMotor.setControl(turretDutyCycleOutRequest.withOutput(SLOW_CALIBRATION_SPEED));
            }
            // Otherwise (so if right sensor isn't tripped, and either the initial calibration timestamp hasn't been initialized or it hasn't reached the stalling time)...
            else {
                // Move counterclockwise quickly
                rotationAngleMotor.setControl(turretDutyCycleOutRequest.withOutput(FAST_CALIBRATION_SPEED));
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
        rotationAngleMotorOffsetRotations = rotationAngleMotorOffsetRotations - beamBreakPositionMotorRotations;
        rotationAngleMotor.setControl(turretDutyCycleOutRequest.withOutput(0));
        isCalibrated = true;
    }

    private void setAutomaticRotationAngle() {
        if(wantedState != SHOOTER_STATE.AUTOMATIC) return;

        Rotation2d robotCentricWantedAngle =
            currentTarget.getPosition().toTranslation2d()
                .minus(BaseRobotState.robotPose.getTranslation())
                .getAngle()
                .minus(BaseRobotState.robotPose.getRotation());
    }

    private void setLaunchMotorVelocities(double wantedVelocity) {
        topLaunchMotor.setControl(topLaunchMotorVelocityRequest.withVelocity(wantedVelocity));
        bottomLaunchMotor.setControl(bottomLaunchMotorVelocityRequest.withVelocity(wantedVelocity));
    }

    private void setLaunchAngle(double wantedAngleDegrees) {
        double rotations = wantedAngleDegrees / 360;
        launchAngleMotor.setControl(launchAnglePositionRequest.withPosition(rotations));
    }

    public SHOOTER_STATE getWantedState() {
        return wantedState;
    }

    /**
     * Sends a position request to the {@link #rotationAngleMotor} based on the passed in
     * robot-relative angle in counterclockwise positive degrees.
     *
     * @param wantedAngleDegrees The desired robot-relative angle to point the turret at, in
     *                           degrees counterclockwise from forward.
     */
    private void setRotationAngle(double wantedAngleDegrees) {
        if (isCalibrated) {
            double wantedTurretRotations = wantedAngleDegrees / 360;

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
            double clampedMotorRotations = MathUtil.clamp(
                wantedMotorRotations,
                -SECOND_LOWEST_BEAM_BREAK_TO_ZERO - CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS,
                -SECOND_LOWEST_BEAM_BREAK_TO_ZERO + FAR_DISTANCE_BETWEEN_BEAM_BREAKS + CLOSE_DISTANCE_BETWEEN_BEAM_BREAKS
            );

            rotationAngleMotor.setControl(turretPositionRequest.withPosition(
                clampedMotorRotations + rotationAngleMotorOffsetRotations
            ));
        }
        else {
            GreenLogger.log("Can't set rotation angle of shooter. Rotation not calibrated.");
        }
    }

    /**
     * Gets the {@link Pose2d} representing the turret's current pose on the field.
     *
     * @return The current field-relative {@link Pose2d} of the turret.
     */
    private Pose2d getCurrentTurretPose2d() {
        return new Pose2d(
            BaseRobotState.robotPose.getTranslation(),
            BaseRobotState.robotPose.getRotation().plus(
                Rotation2d.fromRotations(
                    (rotationAngleMotor.getMotorPosition() - rotationAngleMotorOffsetRotations)
                        / MOTOR_ROTATIONS_PER_TURRET_ROTATION
                )
            )
        );
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }
}
