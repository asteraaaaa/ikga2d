package com.csc4702.ikga2d;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class GeneticAlgorithmTest {
    private final RobotArm2D robot = new RobotArm2D(1.0, 1.0);

    private GAParameters params(long seed) {
        return IKController.defaultParameters().withSeed(seed);
    }

    @Test
    void convergesOnReachableTarget() {
        GeneticAlgorithm.Result r = new GeneticAlgorithm(params(42), robot, 1.2, 0.8).run();
        assertTrue(r.targetReachable());
        assertTrue(r.converged());
        assertTrue(r.best().getFitness() < 0.01);
    }

    @Test
    void sameSeedIsReproducible() {
        GeneticAlgorithm.Result a = new GeneticAlgorithm(params(7), robot, -0.5, 1.1).run();
        GeneticAlgorithm.Result b = new GeneticAlgorithm(params(7), robot, -0.5, 1.1).run();
        assertEquals(a.best().getTheta1(), b.best().getTheta1());
        assertEquals(a.best().getTheta2(), b.best().getTheta2());
        assertEquals(a.generations(), b.generations());
    }

    @Test
    void flagsUnreachableTargetAndStopsEarly() {
        GeneticAlgorithm.Result r = new GeneticAlgorithm(params(1), robot, 3.0, 0.0).run();
        assertFalse(r.targetReachable());
        assertFalse(r.converged());
        assertTrue(r.generations() < 200, "should stop on stagnation");
        assertEquals(1.0, r.best().getFitness(), 0.05); // closest reachable point is at distance 2 -> error ~1
    }

    @Test
    void anglesStayWithinBounds() {
        GeneticAlgorithm.Result r = new GeneticAlgorithm(params(3), robot, 0.3, -1.4).run();
        assertTrue(Math.abs(r.best().getTheta1()) <= Math.PI + 1e-9);
        assertTrue(Math.abs(r.best().getTheta2()) <= Math.PI + 1e-9);
    }

    @Test
    void listenerReceivesGenerations() {
        int[] count = {0};
        GeneticAlgorithm ga = new GeneticAlgorithm(params(5), robot, 3.0, 0.0);
        ga.setGenerationListener(s -> count[0]++);
        GeneticAlgorithm.Result r = ga.run();
        assertEquals(r.generations(), count[0]);
    }

    @Test
    void invalidParametersRejected() {
        assertThrows(IllegalArgumentException.class, () -> new GAParameters(1, 0.3, 0.8, 10, -1, 1));
        assertThrows(IllegalArgumentException.class, () -> new GAParameters(10, 1.5, 0.8, 10, -1, 1));
        assertThrows(IllegalArgumentException.class, () -> new GAParameters(10, 0.3, 0.8, 10, 1, 1));
    }
}
