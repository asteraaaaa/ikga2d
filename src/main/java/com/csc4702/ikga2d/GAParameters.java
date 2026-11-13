package com.csc4702.ikga2d;

/**
 * Immutable configuration for the genetic algorithm.
 */
public class GAParameters {
    public static final double DEFAULT_MUTATION_SIGMA = 0.05;
    public static final double DEFAULT_TOLERANCE = 0.01;
    public static final int DEFAULT_STAGNATION_LIMIT = 50;

    private final int populationSize;
    private final double mutationRate;
    private final double crossoverRate;
    private final int maxGenerations;
    private final double minAngle;
    private final double maxAngle;
    private final double mutationSigma;
    private final double tolerance;
    private final int stagnationLimit;
    private final Long seed;

    public GAParameters(int populationSize, double mutationRate, double crossoverRate,
                        int maxGenerations, double minAngle, double maxAngle) {
        this(populationSize, mutationRate, crossoverRate, maxGenerations, minAngle, maxAngle,
                DEFAULT_MUTATION_SIGMA, DEFAULT_TOLERANCE, DEFAULT_STAGNATION_LIMIT, null);
    }

    /**
     * @param mutationSigma   standard deviation (radians) of the Gaussian mutation step
     * @param tolerance       error below which a solution counts as converged
     * @param stagnationLimit generations without improvement before the run stops early
     * @param seed            RNG seed for reproducible runs, or {@code null} for a random seed
     */
    public GAParameters(int populationSize, double mutationRate, double crossoverRate,
                        int maxGenerations, double minAngle, double maxAngle,
                        double mutationSigma, double tolerance, int stagnationLimit, Long seed) {
        if (populationSize < 2) throw new IllegalArgumentException("populationSize must be >= 2");
        if (mutationRate < 0 || mutationRate > 1) throw new IllegalArgumentException("mutationRate must be in [0,1]");
        if (crossoverRate < 0 || crossoverRate > 1) throw new IllegalArgumentException("crossoverRate must be in [0,1]");
        if (maxGenerations < 0) throw new IllegalArgumentException("maxGenerations must be >= 0");
        if (minAngle >= maxAngle) throw new IllegalArgumentException("minAngle must be < maxAngle");
        if (mutationSigma < 0) throw new IllegalArgumentException("mutationSigma must be >= 0");
        if (tolerance < 0) throw new IllegalArgumentException("tolerance must be >= 0");
        if (stagnationLimit < 1) throw new IllegalArgumentException("stagnationLimit must be >= 1");
        this.populationSize = populationSize;
        this.mutationRate = mutationRate;
        this.crossoverRate = crossoverRate;
        this.maxGenerations = maxGenerations;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.mutationSigma = mutationSigma;
        this.tolerance = tolerance;
        this.stagnationLimit = stagnationLimit;
        this.seed = seed;
    }

    public GAParameters withSeed(Long newSeed) {
        return new GAParameters(populationSize, mutationRate, crossoverRate, maxGenerations,
                minAngle, maxAngle, mutationSigma, tolerance, stagnationLimit, newSeed);
    }

    public int getPopulationSize() { return populationSize; }
    public double getMutationRate() { return mutationRate; }
    public double getCrossoverRate() { return crossoverRate; }
    public int getMaxGenerations() { return maxGenerations; }
    public double getMinAngle() { return minAngle; }
    public double getMaxAngle() { return maxAngle; }
    public double getMutationSigma() { return mutationSigma; }
    public double getTolerance() { return tolerance; }
    public int getStagnationLimit() { return stagnationLimit; }
    /** @return the seed, or {@code null} if runs should be non-deterministic */
    public Long getSeed() { return seed; }
}
