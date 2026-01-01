package com.csc4702.ikga2d;

import java.util.Objects;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Represents one candidate solution with two joint angles.
 */
public class Individual {
    private double theta1;
    private double theta2;
    private double fitness;

    public Individual(double theta1, double theta2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
    }

    public double getTheta1() {
        return theta1;
    }

    public double getTheta2() {
        return theta2;
    }

    public double getFitness() {
        return fitness;
    }

    public void setTheta1(double theta1) {
        this.theta1 = theta1;
    }

    public void setTheta2(double theta2) {
        this.theta2 = theta2;
    }

    public void setFitness(double fitness) {
        this.fitness = fitness;
    }

    /**
     * Computes Euclidean distance between end effector and target.
     */
    public void evaluateFitness(RobotArm2D robot, double targetX, double targetY) {
        double[] ee = robot.computeEndEffector(theta1, theta2);
        double dx = ee[0] - targetX;
        double dy = ee[1] - targetY;
        this.fitness = Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public Individual clone() {
        return new Individual(theta1, theta2);
    }

    @Override
    public String toString() {
        return String.format("Individual(theta1=%.3f, theta2=%.3f, fitness=%.5f)", theta1, theta2, fitness);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Individual other)) {
            return false;
        }
        return Double.compare(other.theta1, theta1) == 0
                && Double.compare(other.theta2, theta2) == 0
                && Double.compare(other.fitness, fitness) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(theta1, theta2, fitness);
    }

    public static Individual randomWithinBounds(double minAngle, double maxAngle) {
        ThreadLocalRandom rng = ThreadLocalRandom.current();
        double theta1 = rng.nextDouble(minAngle, maxAngle);
        double theta2 = rng.nextDouble(minAngle, maxAngle);
        return new Individual(theta1, theta2);
    }
}