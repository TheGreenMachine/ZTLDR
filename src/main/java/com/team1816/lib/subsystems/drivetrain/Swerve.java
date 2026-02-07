package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.Singleton;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
public class Swerve extends SubsystemBase implements ITestableSubsystem {

    private final IDrivetrain drivetrain = Singleton.CreateSubSystem(CTRESwerveDrivetrainImpl.class);
    private final CommandXboxController controller;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);    // forward/back
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);    // strafe
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);  // rotation
    private static double maxAngularRate = 0;
    private SwerveDrivetrain.SwerveDriveState swerveDriveState;

    /* Robot swerve drive state */
    private static StructPublisher<Pose2d> drivePose;
    private static StructPublisher<ChassisSpeeds> driveSpeeds;
    private static StructArrayPublisher<SwerveModuleState> driveModuleStates;
    private static StructArrayPublisher<SwerveModuleState> driveModuleTargets;
    private static StructArrayPublisher<SwerveModulePosition> driveModulePositions;
    private static DoublePublisher driveTimestamp;
    private static DoublePublisher driveOdometryFrequency;
    private static DoubleArrayPublisher fieldPub;
    private static StringPublisher fieldTypePub;
    private static final double[] poseArray = new double[3];

    private SWERVE_STATE wantedState = SWERVE_STATE.SWERVE_IDLE;

    public Swerve(CommandXboxController controller) {
        this.controller = controller;
        var kinematics = drivetrain.getKinematicsConfig();
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // setup teleop drivetrain command
        maxAngularRate = RotationsPerSecond.of(kinematics.maxAngularRate).in(RadiansPerSecond);

        NetworkTable netTable;
        netTable = NetworkTableInstance.getDefault().getTable("");
        driveSpeeds = netTable.getStructTopic(IDrivetrain.NAME + "/Speeds", ChassisSpeeds.struct).publish();
        driveModuleStates = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModuleStates", SwerveModuleState.struct).publish();
        driveModuleTargets = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModuleTargets", SwerveModuleState.struct).publish();
        driveModulePositions = netTable.getStructArrayTopic(IDrivetrain.NAME + "/ModulePositions", SwerveModulePosition.struct).publish();
        driveTimestamp = netTable.getDoubleTopic(IDrivetrain.NAME + "/Timestamp").publish();
        driveOdometryFrequency = netTable.getDoubleTopic(IDrivetrain.NAME + "/OdometryFrequency").publish();
        // name must be Robot for elastic to show as robot in UI
        fieldPub = netTable.getDoubleArrayTopic("Field/Robot").publish();
        fieldTypePub = netTable.getStringTopic("Field/.type").publish();
    }

    public enum SWERVE_STATE {
        SWERVE_DRIVE,
        SWERVE_IDLE
    }

    @Override
    public void periodic() {
        readFromHardware();
        applyStates();
    }

    @Override
    public void readFromHardware() {
        // Get state to use locally
        swerveDriveState = drivetrain.getState();
        // Publish the state to the base robot state
        BaseRobotState.robotPose = swerveDriveState.Pose;
        BaseRobotState.robotSpeeds = swerveDriveState.Speeds;
        processSwerveState();
    }

    private SwerveRequest GetSwerverCommand(SwerveRequest.FieldCentric drive) {

        // 1. Get raw joystick values (-1.0 to +1.0)
        double rawX    = -controller.getLeftY();    // forward/back  (negative because forward is usually negative Y)
        double rawY    = -controller.getLeftX();    // strafe left/right
        double rawRot  = -controller.getRightX();   // rotation

        // 2. Deadband (remove drift)
        rawX   = Math.abs(rawX)   < 0.08 ? 0 : rawX;
        rawY   = Math.abs(rawY)   < 0.08 ? 0 : rawY;
        rawRot = Math.abs(rawRot) < 0.08 ? 0 : rawRot;

        // 3. Cube the inputs → insane precision at low speed, full power at full stick
        double x    = rawX   * rawX   * rawX;
        double y    = rawY   * rawY   * rawY;
        double rot  = rawRot * rawRot * rawRot;

        // 4. Slew rate limit → buttery smooth acceleration
        x   = xLimiter.calculate(x);
        y   = yLimiter.calculate(y);
        rot = rotLimiter.calculate(rot);

        // 5. Slow mode (right bumper = precision mode)
        if (controller.rightBumper().getAsBoolean()) {
            x   *= 0.35;
            y   *= 0.35;
            rot *= 0.45;
        }

        return drive.withVelocityX(x * drivetrain.maxSpd) // Drive forward with negative Y (forward)
            .withVelocityY(y * drivetrain.maxSpd) // Drive left with negative X (left)
            .withRotationalRate(rot * maxAngularRate); // Drive counterclockwise with negative X (left)
    }

    private void applyStates() {
        switch (wantedState) {
            case SWERVE_DRIVE:
                SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                drivetrain.setSwerveState(GetSwerverCommand(drive));
                break;
            case SWERVE_IDLE:
            default:
                drivetrain.setSwerveState(new SwerveRequest.Idle());
                break;
        }
    }

    public void setWantedState(SWERVE_STATE state) {
        this.wantedState = state;
    }

    private void processSwerveState() {
        driveSpeeds.set(swerveDriveState.Speeds);
        if(swerveDriveState.ModuleStates != null) {
            driveModuleStates.set(swerveDriveState.ModuleStates);
            driveModuleTargets.set(swerveDriveState.ModuleTargets);
            driveModulePositions.set(swerveDriveState.ModulePositions);
        }
        driveTimestamp.set(swerveDriveState.Timestamp);
        driveOdometryFrequency.set(1.0 / swerveDriveState.OdometryPeriod);
        processPose2d(swerveDriveState.Pose);
    }

    public static void processPose2d(Pose2d pose) {
        // update Field NOTE: this format is deprecated for advantage scope but no support in simulator yet
        poseArray[0] = pose.getX();
        poseArray[1] = pose.getY();
        poseArray[2] = pose.getRotation().getDegrees();
        fieldTypePub.set("Field2d");
        fieldPub.set(poseArray);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs)
    {
        drivetrain.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs
        );
    }
}
