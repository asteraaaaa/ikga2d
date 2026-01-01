package com.csc4702.ikga2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Core GA engine that evolves joint angles to minimize the positional error.
 */
public class GeneticAlgorithm {
    private final List<Individual> population = new ArrayList<>();
    private final GAParameters parameters;
    private final RobotArm2D robot;
    private final double targetX;
    private final double targetY;

    public GeneticAlgorithm(GAParameters parameters, RobotArm2D robot, double targetX, double targetY) {
        this.parameters = parameters;
        this.robot = robot;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    public Individual run() {
        initializePopulation();
        evaluatePopulation();
        Individual best = getBestIndividual();

        for (int generation = 1; generation <= parameters.getMaxGenerations(); generation++) {
            List<Individual> nextGeneration = new ArrayList<>();
            // Elitism: carry the best individual forward
            nextGeneration.add(best.clone());

            while (nextGeneration.size() < parameters.getPopulationSize()) {
                Individual parent1 = selectParent();
                Individual parent2 = selectParent();
                Individual child = crossover(parent1, parent2);
                mutate(child);
                child.evaluateFitness(robot, targetX, targetY);
                nextGeneration.add(child);
            }

            population.clear();
            population.addAll(nextGeneration);
            best = getBestIndividual();

            printGenerationStats(generation, best);

            if (best.getFitness() < 0.01) {
                break;
            }
        }
        return best;
    }

    private void initializePopulation() {
        population.clear();
        for (int i = 0; i < parameters.getPopulationSize(); i++) {
            Individual individual = Individual.randomWithinBounds(parameters.getMinAngle(), parameters.getMaxAngle());
            population.add(individual);
        }
    }

    private void evaluatePopulation() {
        for (Individual individual : population) {
            individual.evaluateFitness(robot, targetX, targetY);
        }
    }

    private Individual selectParent() {
        int tournamentSize = Math.max(2, parameters.getPopulationSize() / 10);
        List<Individual> tournament = new ArrayList<>();
        ThreadLocalRandom rng = ThreadLocalRandom.current();
        for (int i = 0; i < tournamentSize; i++) {
            tournament.add(population.get(rng.nextInt(population.size())));
        }
        return Collections.min(tournament, Comparator.comparingDouble(Individual::getFitness));
    }

    private Individual crossover(Individual p1, Individual p2) {
        ThreadLocalRandom rng = ThreadLocalRandom.current();
        if (rng.nextDouble() > parameters.getCrossoverRate()) {
            return p1.clone();
        }
        double alpha = rng.nextDouble();
        double theta1 = alpha * p1.getTheta1() + (1 - alpha) * p2.getTheta1();
        double theta2 = alpha * p1.getTheta2() + (1 - alpha) * p2.getTheta2();
        return clamp(new Individual(theta1, theta2));
    }

    private void mutate(Individual individual) {
        ThreadLocalRandom rng = ThreadLocalRandom.current();
        if (rng.nextDouble() < parameters.getMutationRate()) {
            double noise = rng.nextGaussian() * 0.05; // small perturbation
            individual.setTheta1(individual.getTheta1() + noise);
        }
        if (rng.nextDouble() < parameters.getMutationRate()) {
            double noise = rng.nextGaussian() * 0.05;
            individual.setTheta2(individual.getTheta2() + noise);
        }
        clamp(individual);
    }

    private Individual clamp(Individual individual) {
        individual.setTheta1(Math.max(parameters.getMinAngle(), Math.min(parameters.getMaxAngle(), individual.getTheta1())));
        individual.setTheta2(Math.max(parameters.getMinAngle(), Math.min(parameters.getMaxAngle(), individual.getTheta2())));
        return individual;
    }

    private Individual getBestIndividual() {
        evaluatePopulation();
        return Collections.min(population, Comparator.comparingDouble(Individual::getFitness));
    }

    private void printGenerationStats(int generation, Individual best) {
        double[] position = robot.computeEndEffector(best.getTheta1(), best.getTheta2());
        System.out.printf("Generation %d%n", generation);
        System.out.printf("Best θ1 = %.4f rad, θ2 = %.4f rad%n", best.getTheta1(), best.getTheta2());
        System.out.printf("End Effector = (%.4f, %.4f)%n", position[0], position[1]);
        System.out.printf("Error = %.6f%n%n", best.getFitness());
    }
}