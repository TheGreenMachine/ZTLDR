package com.team1816.season.subsystems;

public class BallisticContraints {
    private double gravity = 9.81;
    private double maxYaw = 350; // in degrees
    private double minYaw = 0; // in degrees
    private double minExitVelocity = 5; // m/s
    private double maxExitVelocity = 30; // m/s
    private double maxPitch = 80; // degrees
    private double minPitch = 10; // degrees
    private double endAngle;
    private double height;
    private double funFactorStagnant;//Factor in case the actual robot is missing despite the calculations being mathematically correct, this one is stationary...
    private double funFactorMobile; //and this one if for the robot moving (might not be constant??)

    public double getGravity() {
        return this.gravity;
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
    public boolean isYawValueValid (double yawValue) {
        return yawValue > 0 && yawValue < 80;
    }
    public boolean isPitchValueValid (double pitchValue) {
        return pitchValue > 0 && pitchValue < 80;
    }
    public boolean isExitVelocityValid (double exitVelocity) {
        return exitVelocity > 5 && exitVelocity < 30;
    }
    private double enterAngle;
    public double getEnterAngle (double enterAngle) {
        this.enterAngle = enterAngle;
        return enterAngle;
    }

}
