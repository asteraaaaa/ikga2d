package com.csc4702.ikga2d;

/**
 * Entry point for the GA-based inverse kinematics solver.
 */
public class IKApplication {
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

        IKController controller = new IKController(targetX, targetY);
        controller.runSimulation();
    }
}