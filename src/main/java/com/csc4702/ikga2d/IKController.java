package com.csc4702.ikga2d;

/**
 * Coordinates the genetic algorithm run for a target position.
 */
public class IKController {
    private final double targetX;
    private final double targetY;
    private final RobotArm2D robot;
    private final GeneticAlgorithm ga;

    public IKController(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.robot = new RobotArm2D(1.0, 1.0);

        GAParameters params = new GAParameters(
                80,       // population size
                0.3,      // mutation rate
                0.8,      // crossover rate
                200,      // max generations
                -Math.PI, // min angle
                Math.PI   // max angle
        );
        this.ga = new GeneticAlgorithm(params, robot, targetX, targetY);
    }

    public void runSimulation() {
        System.out.printf("Target position: (%.3f, %.3f)%n", targetX, targetY);
        System.out.println("Running Genetic Algorithm for inverse kinematics...\n");
        Individual best = ga.run();
        displayResult(best);
    }

    private void displayResult(Individual best) {
        double[] position = robot.computeEndEffector(best.getTheta1(), best.getTheta2());
        System.out.println("Final best solution:");
        System.out.printf("θ1 = %.4f rad, θ2 = %.4f rad%n", best.getTheta1(), best.getTheta2());
        System.out.printf("End Effector = (%.4f, %.4f)%n", position[0], position[1]);
        System.out.printf("Error = %.6f%n", best.getFitness());
    }
}