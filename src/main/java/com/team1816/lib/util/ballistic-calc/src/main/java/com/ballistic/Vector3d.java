package com.ballistic;

/**
 * Immutable 3D vector class for ballistic calculations.
 */
public class Vector3d {
    public final double x;
    public final double y;
    public final double z;

    public static final Vector3d ZERO = new Vector3d(0, 0, 0);

    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3d add(Vector3d other) {
        return new Vector3d(x + other.x, y + other.y, z + other.z);
    }

    public Vector3d subtract(Vector3d other) {
        return new Vector3d(x - other.x, y - other.y, z - other.z);
    }

    public Vector3d multiply(double scalar) {
        return new Vector3d(x * scalar, y * scalar, z * scalar);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double horizontalMagnitude() {
        return Math.sqrt(x * x + z * z);
    }

    public Vector3d normalize() {
        double mag = magnitude();
        if (mag == 0) {
            return ZERO;
        }
        return new Vector3d(x / mag, y / mag, z / mag);
    }

    public double dot(Vector3d other) {
        return x * other.x + y * other.y + z * other.z;
    }

    public Vector3d cross(Vector3d other) {
        return new Vector3d(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    /**
     * Returns the horizontal angle (yaw) in radians from positive X axis.
     * Measured counterclockwise when viewed from above.
     */
    public double horizontalAngle() {
        return Math.atan2(z, x);
    }

    @Override
    public String toString() {
        return String.format("Vector3d(%.4f, %.4f, %.4f)", x, y, z);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Vector3d other = (Vector3d) obj;
        return Double.compare(other.x, x) == 0 &&
               Double.compare(other.y, y) == 0 &&
               Double.compare(other.z, z) == 0;
    }

    @Override
    public int hashCode() {
        int result = Double.hashCode(x);
        result = 31 * result + Double.hashCode(y);
        result = 31 * result + Double.hashCode(z);
        return result;
    }
}
