package com.team1816.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class BaseRobotState {
    /**
     * The pose of the robot on the field estimated from odometry and vision data, measured at the
     * robot's center.
     */
    public static Pose2d robotPose = Pose2d.kZero;

    /**
     * The robot-centric speeds of the robot.
     */
    public static ChassisSpeeds robotSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * If you are trying to get the pose of the robot on the field, you probably want to use the
     * robotPose. Only use this if you know what you are doing.
     * <p>
     * A Pose2d representing the "actual" position of the robot in the sim, or the raw odometry
     * pose (without vision estimates) on the real robot. These are the same thing because the sim
     * actual position is intended to be used by vision as the place the cameras simulate seeing
     * from to create simulated camera frames, meaning that it should be a pre-vision position used
     * as the source of truth during simulation, while the raw odometry position is just supposed
     * to be the pre-vision position estimate of the real robot.
     */
    public static Pose2d simActualOrRawOdometryPose = Pose2d.kZero;
}
