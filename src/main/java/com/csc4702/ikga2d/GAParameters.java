package com.csc4702.ikga2d;

/**
 * Configuration holder for the genetic algorithm.
 */
public class GAParameters {
    private final int populationSize;
    private final double mutationRate;
    private final double crossoverRate;
    private final int maxGenerations;
    private final double minAngle;
    private final double maxAngle;

    public GAParameters(int populationSize, double mutationRate, double crossoverRate,
                        int maxGenerations, double minAngle, double maxAngle) {
        this.populationSize = populationSize;
        this.mutationRate = mutationRate;
        this.crossoverRate = crossoverRate;
        this.maxGenerations = maxGenerations;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public int getPopulationSize() {
        return populationSize;
    }

    public double getMutationRate() {
        return mutationRate;
    }

    public double getCrossoverRate() {
        return crossoverRate;
    }

    public int getMaxGenerations() {
        return maxGenerations;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public double getMaxAngle() {
        return maxAngle;
    }
}