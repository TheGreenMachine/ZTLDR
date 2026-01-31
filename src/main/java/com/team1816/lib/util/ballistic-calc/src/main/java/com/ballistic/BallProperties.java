package com.ballistic;

/**
 * Physical properties of the ball being launched.
 */
public class BallProperties {
    private final double diameter;  // meters
    private final double mass;      // kilograms

    /**
     * Creates ball properties.
     *
     * @param diameterMeters Diameter of the ball in meters
     * @param massKg Mass of the ball in kilograms
     */
    public BallProperties(double diameterMeters, double massKg) {
        if (diameterMeters <= 0) {
            throw new IllegalArgumentException("Diameter must be positive");
        }
        if (massKg <= 0) {
            throw new IllegalArgumentException("Mass must be positive");
        }
        this.diameter = diameterMeters;
        this.mass = massKg;
    }

    /**
     * Creates ball properties from inches and kilograms.
     *
     * @param diameterInches Diameter in inches
     * @param massKg Mass in kilograms
     * @return BallProperties instance
     */
    public static BallProperties fromInches(double diameterInches, double massKg) {
        return new BallProperties(diameterInches * 0.0254, massKg);
    }

    public double getDiameter() {
        return diameter;
    }

    public double getRadius() {
        return diameter / 2.0;
    }

    public double getMass() {
        return mass;
    }

    @Override
    public String toString() {
        return String.format("BallProperties[diameter=%.4fm (%.2fin), mass=%.3fkg]",
            diameter, diameter / 0.0254, mass);
    }
}
