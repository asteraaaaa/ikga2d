package com.csc4702.ikga2d;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RobotArm2DTest {
    private final RobotArm2D robot = new RobotArm2D(1.0, 1.0);

    @Test
    void forwardKinematicsStraightArm() {
        double[] p = robot.computeEndEffector(0, 0);
        assertEquals(2.0, p[0], 1e-9);
        assertEquals(0.0, p[1], 1e-9);
    }

    @Test
    void forwardKinematicsRightAngle() {
        double[] p = robot.computeEndEffector(0, Math.PI / 2);
        assertEquals(1.0, p[0], 1e-9);
        assertEquals(1.0, p[1], 1e-9);
    }

    @Test
    void reachability() {
        assertTrue(robot.isReachable(1.2, 0.8));
        assertTrue(robot.isReachable(2.0, 0.0));
        assertFalse(robot.isReachable(2.5, 0.0));
        assertFalse(new RobotArm2D(2.0, 1.0).isReachable(0.2, 0.0));
    }

    @Test
    void rejectsNonPositiveLinks() {
        assertThrows(IllegalArgumentException.class, () -> new RobotArm2D(0, 1));
    }
}
