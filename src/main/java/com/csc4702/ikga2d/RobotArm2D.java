package com.csc4702.ikga2d;

/**
 * Simple 2-DOF planar robot arm providing forward kinematics.
 */
public class RobotArm2D {
    private final double link1Length;
    private final double link2Length;

    public RobotArm2D(double link1Length, double link2Length) {
        if (link1Length <= 0 || link2Length <= 0) {
            throw new IllegalArgumentException("Link lengths must be positive");
        }
        this.link1Length = link1Length;
        this.link2Length = link2Length;
    }

    public double getLink1Length() { return link1Length; }
    public double getLink2Length() { return link2Length; }

    /** Farthest distance from the base the end effector can reach. */
    public double getMaxReach() { return link1Length + link2Length; }

    /** Closest distance to the base the end effector can reach. */
    public double getMinReach() { return Math.abs(link1Length - link2Length); }

    /** @return true if (x, y) lies within the arm's annular workspace. */
    public boolean isReachable(double x, double y) {
        double d = Math.hypot(x, y);
        double eps = 1e-9;
        return d <= getMaxReach() + eps && d >= getMinReach() - eps;
    }

    /**
     * Computes the end effector position using forward kinematics.
     *
     * @return array containing [x, y]
     */
    public double[] computeEndEffector(double theta1, double theta2) {
        return new double[]{
                link1Length * Math.cos(theta1) + link2Length * Math.cos(theta1 + theta2),
                link1Length * Math.sin(theta1) + link2Length * Math.sin(theta1 + theta2)
        };
    }

    /** Position of the elbow joint, useful for drawing. */
    public double[] computeElbow(double theta1) {
        return new double[]{link1Length * Math.cos(theta1), link1Length * Math.sin(theta1)};
    }
}
