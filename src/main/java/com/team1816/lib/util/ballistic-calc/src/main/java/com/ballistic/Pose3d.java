package com.ballistic;

/**
 * Represents a 3D pose (position + orientation).
 * Orientation is represented as Euler angles (roll, pitch, yaw) in radians.
 */
public class Pose3d {
    private final Vector3d position;
    private final double roll;   // Rotation around forward axis (X)
    private final double pitch;  // Rotation around lateral axis (Z) - nose up/down
    private final double yaw;    // Rotation around vertical axis (Y) - heading

    public Pose3d(Vector3d position, double roll, double pitch, double yaw) {
        this.position = position;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public Pose3d(double x, double y, double z, double roll, double pitch, double yaw) {
        this(new Vector3d(x, y, z), roll, pitch, yaw);
    }

    public Vector3d getPosition() {
        return position;
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    /**
     * Gets the forward direction vector based on current orientation.
     */
    public Vector3d getForwardVector() {
        double x = Math.cos(pitch) * Math.cos(yaw);
        double y = Math.sin(pitch);
        double z = Math.cos(pitch) * Math.sin(yaw);
        return new Vector3d(x, y, z);
    }

    @Override
    public String toString() {
        return String.format("Pose3d(pos=%s, roll=%.2f°, pitch=%.2f°, yaw=%.2f°)",
            position,
            Math.toDegrees(roll),
            Math.toDegrees(pitch),
            Math.toDegrees(yaw));
    }
}
