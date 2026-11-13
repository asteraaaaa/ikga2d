package com.csc4702.ikga2d;

/**
 * Console runner for the GA-based inverse kinematics solver.
 * Usage: IKConsole [targetX targetY]
 */
public class IKConsole {
    public static void main(String[] args) {
        double targetX = 1.2;
        double targetY = 0.8;
        if (args.length == 2) {
            try {
                targetX = Double.parseDouble(args[0]);
                targetY = Double.parseDouble(args[1]);
            } catch (NumberFormatException e) {
                System.err.println("Invalid arguments. Using default target (1.2, 0.8).");
            }
        }

        RobotArm2D robot = new RobotArm2D(1.0, 1.0);
        System.out.printf("Target position: (%.3f, %.3f)%n", targetX, targetY);
        if (!robot.isReachable(targetX, targetY)) {
            System.out.printf("Warning: target is outside the reachable workspace "
                    + "[%.2f, %.2f]; the closest reachable pose will be reported.%n",
                    robot.getMinReach(), robot.getMaxReach());
        }

        GeneticAlgorithm ga = new GeneticAlgorithm(IKController.defaultParameters(), robot, targetX, targetY);
        ga.setGenerationListener(s -> {
            if (s.generation() % 10 == 0) {
                System.out.printf("Gen %3d  error = %.6f%n", s.generation(), s.best().getFitness());
            }
        });
        GeneticAlgorithm.Result r = ga.run();

        double[] p = robot.computeEndEffector(r.best().getTheta1(), r.best().getTheta2());
        System.out.printf("%nFinished after %d generations (%s)%n", r.generations(),
                r.converged() ? "converged" : "did not converge");
        System.out.printf("θ1 = %.4f rad, θ2 = %.4f rad%n", r.best().getTheta1(), r.best().getTheta2());
        System.out.printf("End Effector = (%.4f, %.4f)%n", p[0], p[1]);
        System.out.printf("Error = %.6f%n", r.best().getFitness());
    }
}
