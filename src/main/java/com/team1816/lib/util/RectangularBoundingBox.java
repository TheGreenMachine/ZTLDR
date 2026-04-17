package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class RectangularBoundingBox  {
    private final Translation2d bottomLeftCorner;
    private final Translation2d topRightCorner;

    public RectangularBoundingBox(Translation2d corner1, Translation2d corner2) {
        this.bottomLeftCorner = new Translation2d(Math.min(corner1.getX(), corner2.getX()), Math.min(corner1.getY(), corner2.getY()));
        this.topRightCorner = new Translation2d(Math.max(corner1.getX(), corner2.getX()), Math.max(corner1.getY(), corner2.getY()));
    }

    public boolean withinBounds(Translation2d trans) {
        return this.withinBounds(trans.getX(), trans.getY());
    }

    public boolean withinBounds(double xMeters, double yMeters) {
        return
            xMeters >= this.bottomLeftCorner.getX()
                && xMeters <= this.topRightCorner.getX()
                && yMeters >= this.bottomLeftCorner.getY()
                && yMeters <= this.topRightCorner.getY()
            ;
    }
}
