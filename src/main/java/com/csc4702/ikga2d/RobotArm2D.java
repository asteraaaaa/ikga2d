package com.csc4702.ikga2d;

/**
 * Simple 2-DOF planar robot arm providing forward kinematics.
 */
public class RobotArm2D {
    private final double link1Length;
    private final double link2Length;

    public RobotArm2D(double link1Length, double link2Length) {
        this.link1Length = link1Length;
        this.link2Length = link2Length;
    }

    public double getLink1Length() {
        return link1Length;
    }

    public double getLink2Length() {
        return link2Length;
    }

    /**
     * Computes the end effector position using forward kinematics.
     *
     * @param theta1 joint 1 angle in radians
     * @param theta2 joint 2 angle in radians
     * @return array containing [x, y]
     */
    public double[] computeEndEffector(double theta1, double theta2) {
        double x = link1Length * Math.cos(theta1) + link2Length * Math.cos(theta1 + theta2);
        double y = link1Length * Math.sin(theta1) + link2Length * Math.sin(theta1 + theta2);
        return new double[]{x, y};
    }
}