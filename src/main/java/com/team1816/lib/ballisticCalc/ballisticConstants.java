package com.team1816.lib.ballisticCalc;

public class ballisticConstants {
    /**
     * These are where our constants live. We'll define them as we learn what they are
     */
    private double gravity = 9.8;
    private double endAngle;
    private double height;
    private double funFactorStagnant;//Factor in case the actual robot is missing despite the calculations being mathematically correct, this one is stationary...
    private double funFactorMobile; //and this one if for the robot moving (might not be constant??)

    public double getGravity(double gravity) {
        this.gravity = gravity;

        return gravity;
    }
    public double getEndAngle(double endAngle) {
        this.endAngle = endAngle;

        return endAngle;
    }
    public double getHeight(double height) {
        this.height = height;

        return height;
    }
    public double getFunFactorStagnant(double funFactor) {
        this.funFactorStagnant = funFactor;

        return funFactor;
    }
    public double getFunFactorMobile(double funFactor) {
        this.funFactorMobile = funFactor;

        return funFactor;
    }


}
