package org.firstinspires.ftc.teamcode.util;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

// LUT (Look Up Table)
public class TurretLUT {
    // TreeMap automatically sorts entries by Key (Distance)
    private final TreeMap<Double, ShooterConfiguration> lut = new TreeMap<>();

    public TurretLUT(List<Datapoint> datapoints) {
        for (Datapoint p : datapoints) {
            lut.put(p.getDistanceToGoal(), p.getShooterConfiguration());
        }
    }

    /**
     * Calculates the optimal shooting config using Linear Interpolation (Lerp).
     * This looks at the datapoint immediately below and immediately above the input
     * and draws a straight line between them.
     *
     * @param distanceToGoal The current distance to the target
     * @return Interpolated RPM and Hood Position
     */
    public ShooterConfiguration calculate(double distanceToGoal) {
        if (lut.isEmpty()) {
            return new ShooterConfiguration(0, 0);
        }

        if (lut.containsKey(distanceToGoal)) {
            return lut.get(distanceToGoal);
        }

        Map.Entry<Double, ShooterConfiguration> lower = lut.floorEntry(distanceToGoal);
        Map.Entry<Double, ShooterConfiguration> upper = lut.ceilingEntry(distanceToGoal);

        if (lower == null) return upper.getValue(); // We are closer than the closest point
        if (upper == null) return lower.getValue(); // We are further than the furthest point

        double lowerDistanceToGoal = lower.getKey();
        double upperDistanceToGoal = upper.getKey();

        if (Math.abs(upperDistanceToGoal - lowerDistanceToGoal) < 0.001) {
            return lower.getValue();
        }

        // 0: lower point, 1: upper point
        double distanceBetweenPoints = (distanceToGoal - lowerDistanceToGoal) / (upperDistanceToGoal - lowerDistanceToGoal);

        double lerpedRPM = lerp(lower.getValue().flywheelRPM, upper.getValue().flywheelRPM, distanceBetweenPoints);
        double lerpedHood = lerp(lower.getValue().hoodServoPosition, upper.getValue().hoodServoPosition, distanceBetweenPoints);

        return new ShooterConfiguration(lerpedRPM, lerpedHood);
    }

    private double lerp(double start, double end, double t) {
        return start + t * (end - start);
    }

    public static class Datapoint {
        private final double distanceToGoal;
        private final ShooterConfiguration shooterConfiguration;

        public Datapoint(double distanceToGoal, ShooterConfiguration shooterConfiguration) {
            this.distanceToGoal = distanceToGoal;
            this.shooterConfiguration = shooterConfiguration;
        }

        public double getDistanceToGoal() {
            return distanceToGoal;
        }

        public ShooterConfiguration getShooterConfiguration() {
            return shooterConfiguration;
        }
    }

    public static class ShooterConfiguration {
        private final double flywheelRPM;
        private final double hoodServoPosition;

        public ShooterConfiguration(double flywheelRPM, double hoodServoPosition) {
            this.flywheelRPM = flywheelRPM;
            this.hoodServoPosition = hoodServoPosition;
        }

        public double getHoodServoPosition() {
            return hoodServoPosition;
        }

        public double getFlywheelRPM() {
            return flywheelRPM;
        }
    }
}


