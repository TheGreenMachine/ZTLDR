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
}
