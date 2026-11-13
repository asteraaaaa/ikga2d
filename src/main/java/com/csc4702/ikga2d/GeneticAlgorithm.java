package com.csc4702.ikga2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Consumer;

/**
 * Core GA engine that evolves joint angles to minimize the positional error.
 */
public class GeneticAlgorithm {
    /** BLX-alpha extension: children may fall slightly outside the parents' range. */
    private static final double BLX_ALPHA = 0.5;
    private static final double IMPROVEMENT_EPS = 1e-9;

    /** Snapshot reported to the listener after each generation. */
    public record GenerationStats(int generation, Individual best, double[] endEffector) {}

    /** Outcome of a run. */
    public record Result(Individual best, int generations, boolean targetReachable, boolean converged) {}

    private final List<Individual> population = new ArrayList<>();
    private final GAParameters parameters;
    private final RobotArm2D robot;
    private final double targetX;
    private final double targetY;
    private final Random rng;
    private Consumer<GenerationStats> listener = s -> { };

    public GeneticAlgorithm(GAParameters parameters, RobotArm2D robot, double targetX, double targetY) {
        this.parameters = parameters;
        this.robot = robot;
        this.targetX = targetX;
        this.targetY = targetY;
        this.rng = parameters.getSeed() == null ? new Random() : new Random(parameters.getSeed());
    }

    /** Registers a callback invoked after every generation (default: none, so runs are silent). */
    public void setGenerationListener(Consumer<GenerationStats> listener) {
        this.listener = listener == null ? s -> { } : listener;
    }

    public Result run() {
        boolean reachable = robot.isReachable(targetX, targetY);
        initializePopulation();
        Individual best = findBest().copy();
        boolean converged = best.getFitness() < parameters.getTolerance();
        int stagnant = 0;
        int generation = 0;

        while (!converged && generation < parameters.getMaxGenerations()) {
            generation++;
            List<Individual> next = new ArrayList<>(parameters.getPopulationSize());
            next.add(best.copy()); // elitism

            while (next.size() < parameters.getPopulationSize()) {
                Individual child = crossover(selectParent(), selectParent());
                mutate(child);
                child.evaluateFitness(robot, targetX, targetY);
                next.add(child);
            }
            population.clear();
            population.addAll(next);

            Individual current = findBest();
            stagnant = current.getFitness() < best.getFitness() - IMPROVEMENT_EPS ? 0 : stagnant + 1;
            best = current.copy();
            converged = best.getFitness() < parameters.getTolerance();

            listener.accept(new GenerationStats(generation, best,
                    robot.computeEndEffector(best.getTheta1(), best.getTheta2())));

            // An unreachable target never converges; stop once progress has stalled.
            if (!converged && stagnant >= parameters.getStagnationLimit()) {
                break;
            }
        }
        return new Result(best, generation, reachable, converged);
    }

    private void initializePopulation() {
        population.clear();
        for (int i = 0; i < parameters.getPopulationSize(); i++) {
            Individual ind = Individual.randomWithinBounds(rng, parameters.getMinAngle(), parameters.getMaxAngle());
            ind.evaluateFitness(robot, targetX, targetY);
            population.add(ind);
        }
    }

    /** Population fitness is always up to date, so no re-evaluation is needed. */
    private Individual findBest() {
        Individual best = population.get(0);
        for (Individual ind : population) {
            if (ind.getFitness() < best.getFitness()) best = ind;
        }
        return best;
    }

    private Individual selectParent() {
        int size = Math.min(population.size(), Math.max(2, parameters.getPopulationSize() / 10));
        Individual winner = null;
        for (int i = 0; i < size; i++) {
            Individual c = population.get(rng.nextInt(population.size()));
            if (winner == null || c.getFitness() < winner.getFitness()) winner = c;
        }
        return winner;
    }

    /** BLX-alpha crossover: samples each gene from the parents' interval extended by alpha on both sides. */
    private Individual crossover(Individual p1, Individual p2) {
        Individual child;
        if (rng.nextDouble() >= parameters.getCrossoverRate()) {
            child = p1.copy();
        } else {
            child = new Individual(blend(p1.getTheta1(), p2.getTheta1()), blend(p1.getTheta2(), p2.getTheta2()));
        }
        return normalize(child);
    }

    private double blend(double a, double b) {
        double lo = Math.min(a, b);
        double hi = Math.max(a, b);
        double d = hi - lo;
        double from = lo - BLX_ALPHA * d;
        double to = hi + BLX_ALPHA * d;
        return from == to ? a : rng.nextDouble(from, to);
    }

    private void mutate(Individual ind) {
        double sigma = parameters.getMutationSigma();
        if (rng.nextDouble() < parameters.getMutationRate()) {
            ind.setTheta1(ind.getTheta1() + rng.nextGaussian() * sigma);
        }
        if (rng.nextDouble() < parameters.getMutationRate()) {
            ind.setTheta2(ind.getTheta2() + rng.nextGaussian() * sigma);
        }
        normalize(ind);
    }

    /** Wraps angles around when the range spans a full turn, otherwise clamps to the joint limits. */
    private Individual normalize(Individual ind) {
        ind.setTheta1(normalizeAngle(ind.getTheta1()));
        ind.setTheta2(normalizeAngle(ind.getTheta2()));
        return ind;
    }

    private double normalizeAngle(double angle) {
        double min = parameters.getMinAngle();
        double max = parameters.getMaxAngle();
        double range = max - min;
        if (range >= 2 * Math.PI - 1e-9) {
            return min + ((angle - min) % range + range) % range;
        }
        return Math.max(min, Math.min(max, angle));
    }
}
