package com.team1816.lib.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1816.lib.hardware.KinematicsConfig;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.subsystems.ITestableSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

import static com.team1816.lib.Singleton.factory;

public interface IDrivetrain extends ITestableSubsystem {
    // setup common variables for use by drivetrain implementations
    String NAME = "drivetrain";

    SubsystemConfig config = factory.getSubsystemConfig(NAME);
    double massKG = config.kinematics.robotMass;
    // this is an approximation assumes mass is evenly spread over robot
    double MOI = (massKG * config.kinematics.wheelbaseWidth * config.kinematics.wheelbaseWidth + config.kinematics.wheelbaseLength * config.kinematics.wheelbaseLength) / 12;
    double whlRad = config.kinematics.wheelRadius;
    double wheelCircumference = 2 * Math.PI * whlRad;
    double maxSpd = (config.kinematics.maxDriveRPS / config.kinematics.driveGearing) * 2 * Math.PI * config.kinematics.wheelRadius;
    double cof = config.kinematics.wheelCOF;
    double gearing = config.kinematics.driveGearing;
    boolean isAllowedToPathPlannerPath = false;

    // Gets configuration for kinematics
    default KinematicsConfig getKinematicsConfig() {
        return config.kinematics;
    }

    boolean IsFieldCentric();

    void resetPose(Pose2d pose);

    SwerveDrivetrain.SwerveDriveState getState();

    default void setSwerveState(SwerveRequest request) {
    }

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
}
