package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.hardware.KinematicsConfig;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static com.team1816.lib.Singleton.factory;

public interface IDrivetrain extends ITestableSubsystem {
    // setup common variables for use by drivetrain implementations
    String NAME = "drivetrain";

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    double massKG = config.kinematics.robotMass;
    double whlRad = config.kinematics.wheelRadius;
    double maxSpd = (config.kinematics.maxDriveRPS / config.kinematics.driveGearing) * 2 * Math.PI * config.kinematics.wheelRadius;
    double cof = config.kinematics.wheelCOF;
    double gearing = config.kinematics.driveGearing;
    double wheelCircumference = 2 * Math.PI * whlRad;

    // this is an approximation assumes mass is evenly spread over robot
    double MOI = (massKG * config.kinematics.wheelbaseWidth * config.kinematics.wheelbaseWidth + config.kinematics.wheelbaseLength * config.kinematics.wheelbaseLength) / 12;

    // Gets configuration for kinematics
    default KinematicsConfig getKinematicsConfig() {
        return config.kinematics;
    }

    boolean IsFieldCentric();

    void resetPose(Pose2d pose);

    /**
     * Teleport the simulated "actual" position of the robot used as the source of truth for
     * simulation to the specified pose. This is used to simulate the robot shifting on the field
     * in a way that wheel odometry does not account for (essentially simulating wheel slippage or
     * other pose loss), in order to test how vision localization corrects for this disruption.
     *
     * @param pose The pose to teleport the sim actual position to.
     */
    void simTeleportRobot(Pose2d pose);

    /**
     * Set up periodic logging for the drivetrain under the specified path.
     *
     * @param logPath The path to log values under.
     */
    void setUpPeriodicLogging(String logPath);

    default void setSwerveState(SwerveRequest request) {}

    // Converts meters to rotations using defined wheel circumference
    // set useGearing to true to account for gearing
    default double metersToRotations(double meters, boolean useGearing) {
        if (useGearing) {
            return meters / wheelCircumference * gearing;
        }
        return meters / wheelCircumference;
    }

    // Converts rotations to meters using defined wheel circumference
    // set useGearing to true to account for gearing
    default double rotationsToMeters(double rotations, boolean useGearing) {
        if (useGearing) {
            return rotations / gearing * wheelCircumference;
        }
        return rotations * wheelCircumference;
    }

    // returns a velocity clamped to the configured maximum
    default double clampVelocity(double velocity) {
        return Math.min(Math.max(velocity, -maxSpd), maxSpd);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds. This
     *                                 should be an FGPA timestamp, which is what PhotonVision
     *                                 returns as part of its {@link
     *                                 org.photonvision.EstimatedRobotPose}.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x
     *                                 position in meters, y position in meters, and heading in
     *                                 radians). Increase these numbers to trust the vision pose
     *                                 measurement less.
     */
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    );

    /**
     * Sets the pose estimator's trust in robot odometry. This might be used to change trust in
     * odometry after an impact with the wall or traversing a bump.
     *
     * @param stateStdDevs Standard deviations of the pose estimate. Increase these numbers to
     *                     trust your state estimate less. This matrix is in the form [x, y,
     *                     theta]ᵀ, with units in meters and radians.
     */
    void setStateStdDevs(Matrix<N3, N1> stateStdDevs);
}
