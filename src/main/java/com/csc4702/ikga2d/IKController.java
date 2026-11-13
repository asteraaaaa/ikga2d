package com.csc4702.ikga2d;

import javafx.fxml.FXML;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;

/**
 * FXML controller: reads the target, runs the GA and draws the resulting arm pose.
 */
public class IKController {
    @FXML private TextField targetXField;
    @FXML private TextField targetYField;
    @FXML private Canvas canvas;
    @FXML private Label resultLabel;

    private final RobotArm2D robot = new RobotArm2D(1.0, 1.0);

    /** Default GA configuration shared by the GUI and console runners. */
    public static GAParameters defaultParameters() {
        return new GAParameters(
                80,       // population size
                0.3,      // mutation rate
                0.8,      // crossover rate
                200,      // max generations
                -Math.PI, // min angle
                Math.PI   // max angle
        );
    }

    @FXML
    private void initialize() {
        draw(null, 0, 0, false);
    }

    @FXML
    private void onRun() {
        double tx;
        double ty;
        try {
            tx = Double.parseDouble(targetXField.getText().trim());
            ty = Double.parseDouble(targetYField.getText().trim());
        } catch (NumberFormatException e) {
            resultLabel.setText("Please enter numeric X and Y values.");
            return;
        }

        GeneticAlgorithm.Result r = new GeneticAlgorithm(defaultParameters(), robot, tx, ty).run();
        double[] ee = robot.computeEndEffector(r.best().getTheta1(), r.best().getTheta2());

        StringBuilder sb = new StringBuilder();
        if (!r.targetReachable()) {
            sb.append(String.format("Target is outside the reachable workspace [%.2f, %.2f]; showing closest pose.%n",
                    robot.getMinReach(), robot.getMaxReach()));
        }
        sb.append(String.format("%s after %d generations%nθ1 = %.4f rad, θ2 = %.4f rad%nEnd effector = (%.4f, %.4f), error = %.6f",
                r.converged() ? "Converged" : "Did not converge", r.generations(),
                r.best().getTheta1(), r.best().getTheta2(), ee[0], ee[1], r.best().getFitness()));
        resultLabel.setText(sb.toString());
        draw(r.best(), tx, ty, true);
    }

    private void draw(Individual pose, double tx, double ty, boolean hasTarget) {
        GraphicsContext g = canvas.getGraphicsContext2D();
        double w = canvas.getWidth();
        double h = canvas.getHeight();
        // Zoom out if the target lies beyond the arm's reach so it stays visible.
        double extent = Math.max(robot.getMaxReach(), hasTarget ? Math.hypot(tx, ty) : 0) + 0.5;
        double scale = Math.min(w, h) / (2 * extent);
        double ox = w / 2;
        double oy = h / 2;

        g.setFill(Color.WHITE);
        g.fillRect(0, 0, w, h);
        g.setStroke(Color.LIGHTGRAY);
        g.strokeLine(0, oy, w, oy);
        g.strokeLine(ox, 0, ox, h);
        double rr = robot.getMaxReach() * scale;
        g.strokeOval(ox - rr, oy - rr, 2 * rr, 2 * rr);

        if (hasTarget) {
            g.setFill(Color.CRIMSON);
            g.fillOval(ox + tx * scale - 5, oy - ty * scale - 5, 10, 10);
        }
        if (pose != null) {
            double[] elbow = robot.computeElbow(pose.getTheta1());
            double[] ee = robot.computeEndEffector(pose.getTheta1(), pose.getTheta2());
            g.setStroke(Color.STEELBLUE);
            g.setLineWidth(4);
            g.strokeLine(ox, oy, ox + elbow[0] * scale, oy - elbow[1] * scale);
            g.strokeLine(ox + elbow[0] * scale, oy - elbow[1] * scale, ox + ee[0] * scale, oy - ee[1] * scale);
            g.setLineWidth(1);
        }
        g.setFill(Color.BLACK);
        g.fillOval(ox - 4, oy - 4, 8, 8);
    }
}
