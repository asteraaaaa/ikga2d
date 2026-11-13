package com.csc4702.ikga2d;

import java.util.Random;

/**
 * One candidate solution with two joint angles.
 * Uses identity equality because the fields are mutable.
 */
public class Individual {
    private double theta1;
    private double theta2;
    private double fitness = Double.POSITIVE_INFINITY;

    public Individual(double theta1, double theta2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
    }

    public double getTheta1() { return theta1; }
    public double getTheta2() { return theta2; }
    public double getFitness() { return fitness; }

    public void setTheta1(double theta1) { this.theta1 = theta1; this.fitness = Double.POSITIVE_INFINITY; }
    public void setTheta2(double theta2) { this.theta2 = theta2; this.fitness = Double.POSITIVE_INFINITY; }

    /** Fitness = Euclidean distance between end effector and target (lower is better). */
    public void evaluateFitness(RobotArm2D robot, double targetX, double targetY) {
        double[] ee = robot.computeEndEffector(theta1, theta2);
        this.fitness = Math.hypot(ee[0] - targetX, ee[1] - targetY);
    }

    /** Copy including the cached fitness. */
    public Individual copy() {
        Individual c = new Individual(theta1, theta2);
        c.fitness = fitness;
        return c;
    }

    @Override
    public String toString() {
        return String.format("Individual(theta1=%.3f, theta2=%.3f, fitness=%.5f)", theta1, theta2, fitness);
    }

    public static Individual randomWithinBounds(Random rng, double minAngle, double maxAngle) {
        return new Individual(rng.nextDouble(minAngle, maxAngle), rng.nextDouble(minAngle, maxAngle));
    }
}
