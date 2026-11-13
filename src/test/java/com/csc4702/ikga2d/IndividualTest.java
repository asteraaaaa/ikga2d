package com.csc4702.ikga2d;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class IndividualTest {
    @Test
    void fitnessIsDistanceToTarget() {
        Individual ind = new Individual(0, 0);
        ind.evaluateFitness(new RobotArm2D(1, 1), 2.0, 1.0);
        assertEquals(1.0, ind.getFitness(), 1e-9);
    }

    @Test
    void copyKeepsGenesAndFitnessButIsDistinct() {
        Individual a = new Individual(0.1, 0.2);
        a.evaluateFitness(new RobotArm2D(1, 1), 1, 1);
        Individual b = a.copy();
        assertNotSame(a, b);
        assertEquals(a.getTheta1(), b.getTheta1());
        assertEquals(a.getFitness(), b.getFitness());
    }

    @Test
    void identityEquality() {
        assertNotEquals(new Individual(1, 2), new Individual(1, 2));
    }

    @Test
    void changingAnglesInvalidatesFitness() {
        Individual a = new Individual(0, 0);
        a.evaluateFitness(new RobotArm2D(1, 1), 2, 0);
        a.setTheta1(1);
        assertEquals(Double.POSITIVE_INFINITY, a.getFitness());
    }
}
