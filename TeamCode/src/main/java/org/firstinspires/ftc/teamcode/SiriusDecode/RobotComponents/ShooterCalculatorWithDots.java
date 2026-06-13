package org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.HashMap;
import java.util.Map;

/**
 * ShooterCalculatorWithDots – discrete grid of positions + equally spaced heading steps.
 * <p>
 * The full circle (360°) is divided into numAngles equal steps.
 * Combined ID = (positionId * numAngles) + angleIndex.
 * All parameters are stored in a single flat map for fast lookup.
 */
public class ShooterCalculatorWithDots {

    public static class DotParameters {
        public final double rpm;
        public final double hoodAngle;
        public final double hoodCompensation;
        public DotParameters(double rpm, double hoodAngle, double hoodCompensation) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
            this.hoodCompensation = hoodCompensation;
        }
    }

    // Position grid
    private final double goalRelX, goalRelY;
    private final double gridWidth, gridHeight;
    public final double step;
    public final int signX, signY;
    public final int xSteps, ySteps;
    private final int totalSpatialDots;
    private final double[] dotRelX, dotRelY;

    // Angle discretisation
    private final int numAngles;          // number of equal steps around the circle
    private final double angleStep;       // 360 / numAngles (degrees)
    private final int totalCombinedDots;

    // Storage: combinedId -> DotParameters
    private final Map<Integer, DotParameters> tunedData = new HashMap<>();

    // ------------------------------------------------------------------------
    // Constructors
    // ------------------------------------------------------------------------
    /**
     * Full constructor with sign control and angle steps.
     *
     * @param goalRelX   goal X relative to robot start (meters)
     * @param goalRelY   goal Y relative to robot start (meters)
     * @param gridWidth  width of spatial grid (meters)
     * @param gridHeight height of spatial grid (meters)
     * @param step       spatial step (meters)
     * @param signX      +1 or -1 (direction of grid from goal)
     * @param signY      +1 or -1
     * @param numAngles  number of discrete heading steps (divides 360° evenly)
     */
    public ShooterCalculatorWithDots(double goalRelX, double goalRelY,
                                     double gridWidth, double gridHeight,
                                     double step, int signX, int signY,
                                     int numAngles) {
        this.goalRelX = goalRelX;
        this.goalRelY = goalRelY;
        this.gridWidth = gridWidth;
        this.gridHeight = gridHeight;
        this.step = step;
        this.signX = signX;
        this.signY = signY;
        this.numAngles = numAngles;
        this.angleStep = 360.0 / numAngles;

        this.xSteps = (int)(gridWidth / step) + 1;
        this.ySteps = (int)(gridHeight / step) + 1;
        this.totalSpatialDots = xSteps * ySteps;
        this.totalCombinedDots = totalSpatialDots * numAngles;

        dotRelX = new double[totalSpatialDots];
        dotRelY = new double[totalSpatialDots];

        // Generate spatial dots
        for (int yIdx = 0; yIdx < ySteps; yIdx++) {
            for (int xIdx = 0; xIdx < xSteps; xIdx++) {
                int spatialId = xIdx + yIdx * xSteps;
                double dxFromGoal = xIdx * step * signX;
                double dyFromGoal = yIdx * step * signY;
                dotRelX[spatialId] = goalRelX + dxFromGoal;
                dotRelY[spatialId] = goalRelY + dyFromGoal;
            }
        }
    }

    /**
     * Convenience constructor (default signs = +1, +1).
     */
    public ShooterCalculatorWithDots(double goalRelX, double goalRelY,
                                     double gridWidth, double gridHeight,
                                     double step, int numAngles) {
        this(goalRelX, goalRelY, gridWidth, gridHeight, step, +1, +1, numAngles);
    }

    // ------------------------------------------------------------------------
    // ID conversion helpers
    // ------------------------------------------------------------------------
    public int getCombinedId(int spatialId, int angleIndex) {
        return spatialId * numAngles + angleIndex;
    }

    /**
     * Returns the nearest spatial dot ID for a given robot position.
     */

    public int FromRowAndColumnGetSpatialId(int row, int column){
        return row * ySteps + column;
    }
    public int getNearestSpatialId(double robotRelX, double robotRelY) {
        int bestId = -1;
        double bestDistSq = Double.POSITIVE_INFINITY;
        for (int id = 0; id < totalSpatialDots; id++) {
            double dx = robotRelX - dotRelX[id];
            double dy = robotRelY - dotRelY[id];
            double distSq = dx*dx + dy*dy;
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestId = id;
            }
        }
        return bestId;
    }

    /**
     * Returns the parameters for the nearest tuned dot by comparing
     * Euclidean distance (spatial) first, then angle as a tiebreaker or secondary match.
     */
    public DotParameters getNearestTunedParametersSimple(double robotRelX, double robotRelY, double robotHeadingDegrees) {
        if (tunedData.isEmpty()) return null;

        int targetAngleIdx = getNearestAngleIndex(robotHeadingDegrees);

        int bestCombinedId = -1;
        double bestDistSq = Double.POSITIVE_INFINITY;
        double bestAngleDelta = 360;

        for (Map.Entry<Integer, DotParameters> entry : tunedData.entrySet()) {
            int combinedId = entry.getKey();
            int spatialId = combinedId / numAngles;
            int tunedAngleIdx = combinedId % numAngles;

            double dx = robotRelX - dotRelX[spatialId];
            double dy = robotRelY - dotRelY[spatialId];
            double distSq = dx*dx + dy*dy;

            double angleDelta = Math.abs(targetAngleIdx - tunedAngleIdx);
            angleDelta = Math.min(angleDelta, numAngles - angleDelta); // circular distance in index space

            // Prioritize: if distance is significantly better, take it
            // If distance is roughly the same, pick better angle match
            if (bestCombinedId == -1) {
                bestCombinedId = combinedId;
                bestDistSq = distSq;
                bestAngleDelta = angleDelta;
            } else if (distSq < bestDistSq - 0.01) { // significantly closer
                bestCombinedId = combinedId;
                bestDistSq = distSq;
                bestAngleDelta = angleDelta;
            } else if (Math.abs(distSq - bestDistSq) <= 0.01 && angleDelta < bestAngleDelta) {
                // similar distance, better angle match
                bestCombinedId = combinedId;
                bestAngleDelta = angleDelta;
            }
        }

        return bestCombinedId == -1 ? null : tunedData.get(bestCombinedId);
    }

    /**
     * Returns the angle index (0 … numAngles-1) for a given robot heading.
     * The angle step is fixed: index = round(heading / angleStep) modulo numAngles.
     */
    public int getNearestAngleIndex(double robotHeadingDegrees) {
        // Normalise to [0, 360)
        double heading = robotHeadingDegrees % 360.0;
        if (heading < 0) heading += 360.0;
        int idx = (int) Math.round(heading / angleStep);
        idx = idx % numAngles;  // ensures 0..numAngles-1
        return idx;
    }

    // ------------------------------------------------------------------------
    // Tuning API
    // ------------------------------------------------------------------------
    /**
     * Tune using robot position + heading (snaps to nearest spatial dot and angle step).
     */
    public void setParameters(double robotRelX, double robotRelY,
                              double robotHeadingDegrees,
                              double rpm, double hoodAngle, double hoodCompensation) {
        int spatialId = getNearestSpatialId(robotRelX, robotRelY);
        int angleIdx = getNearestAngleIndex(robotHeadingDegrees);
        int combinedId = getCombinedId(spatialId, angleIdx);
        tunedData.put(combinedId, new DotParameters(rpm, hoodAngle, hoodCompensation));
    }

    /**
     * Tune directly using a combined ID (spatialId * numAngles + angleIndex).
     */
    public void setParameters(int combinedId, double rpm, double hoodAngle, double hoodCompensation) {
        tunedData.put(combinedId, new DotParameters(rpm, hoodAngle, hoodCompensation));
    }

    /**
     * Tune using separate spatial ID and angle index.
     */
    public void setParameters(int spatialId, int angleIndex,
                              double rpm, double hoodAngle, double hoodCompensation) {
        int combinedId = getCombinedId(spatialId, angleIndex);
        tunedData.put(combinedId, new DotParameters(rpm, hoodAngle, hoodCompensation));
    }

    // ------------------------------------------------------------------------
    // Runtime Lookup
    // ------------------------------------------------------------------------
    /**
     * Returns the parameters for the nearest spatial dot and nearest discrete angle step.
     */

    public DotParameters getNearestParameters(double robotRelX, double robotRelY, double robotHeadingDegrees) {
        if (tunedData.isEmpty()) return null;
        int spatialId = getNearestSpatialId(robotRelX, robotRelY);
        int angleIdx = getNearestAngleIndex(robotHeadingDegrees);
        int combinedId = getCombinedId(spatialId, angleIdx);
        return tunedData.get(combinedId);
    }

    /**
     * Returns the combined ID for debugging.
     */
    public int getNearestCombinedId(double robotRelX, double robotRelY, double robotHeadingDegrees) {
        int spatialId = getNearestSpatialId(robotRelX, robotRelY);
        int angleIdx = getNearestAngleIndex(robotHeadingDegrees);
        return getCombinedId(spatialId, angleIdx);
    }

    // ------------------------------------------------------------------------
    // Utility methods
    // ------------------------------------------------------------------------
    public SparkFunOTOS.Pose2D getPoseFromCombinedId(int combinedId) {
        if (combinedId < 0 || combinedId >= totalCombinedDots) {
            throw new IllegalArgumentException("Invalid combined ID: " + combinedId);
        }
        int spatialId = combinedId / numAngles;
        int angleIdx = combinedId % numAngles;
        double heading = angleIdx * angleStep;  // in degrees
        return new SparkFunOTOS.Pose2D(dotRelX[spatialId], dotRelY[spatialId], Math.toRadians(heading));
    }

    public int getTotalSpatialDots() { return totalSpatialDots; }
    public int getNumAngles() { return numAngles; }
    public int getTotalCombinedDots() { return totalCombinedDots; }
    public double getAngleStep() { return angleStep; }
    public void clearAll() { tunedData.clear(); }

    public void printTunedDots() {
        RobotInitializers.Dashtelemetry.addData("Tuned combined dots", "%d / %d", tunedData.size(), totalCombinedDots);
        for (Map.Entry<Integer, DotParameters> entry : tunedData.entrySet()) {
            int combinedId = entry.getKey();
            int spatialId = combinedId / numAngles;
            int angleIdx = combinedId % numAngles;
            double angle = angleIdx * angleStep;
            double x = dotRelX[spatialId];
            double y = dotRelY[spatialId];
            DotParameters p = entry.getValue();
            RobotInitializers.Dashtelemetry.addData(
                    String.format("CombID %d (spatial %d, angle %.1f°) at (%.2f,%.2f)", combinedId, spatialId, angle, x, y),
                    "RPM=%.0f Hood=%.1f Comp=%.1f", p.rpm, p.hoodAngle, p.hoodCompensation);
        }
    }
}