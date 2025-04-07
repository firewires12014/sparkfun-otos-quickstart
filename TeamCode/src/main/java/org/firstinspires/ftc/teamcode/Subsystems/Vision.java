package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

public class Vision {
    public static double xTranslate = 42;
    public static double yTranslate = 24;
    public static double yOffset = 25;
    public static double xOffset = 2.6;

    // Color that it is NOT looking for
    public String color = "red";

    public Limelight3A limelight;

    public static double[][] H = new double[][] {
            {-1.97539577e+01, -2.15267488e+01,  3.83024175e+03},
            {-3.50811062e+00, -6.66541018e+01,  6.20935580e+03},
            {-6.93140729e-03, -7.92889642e-02,  1.00000000e+00}
    };

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void setBlue() {
        color = "blue";
    }

    public void setRed() {
        color = "red";
    }

    public void stop() {
        limelight.shutdown();
    }

    public ArrayList<Object> getBlock() {
        return getBlock(false);
    }

    public ArrayList<Object> getBlock(boolean fullRange) {
        LLResult result = limelight.getLatestResult();

        double[] outputPosition = new double[]{0, 4}; // default position

        String s_color = "?";

        if (result != null) {
            List<LLResultTypes.DetectorResult> results = result.getDetectorResults();

            // Search through all samples
//            for(LLResultTypes.DetectorResult dr : results) {
//                double[] centroid = getCentroid(dr.getTargetCorners());
//
//                double[] updatedCentroid = applyHomography(H, centroid[0], centroid[1]);
//
//                double newX = updatedCentroid[0]/xTranslate - xOffset;
//                double newY = yOffset - updatedCentroid[1]/yTranslate;
//
//                outputPosition = new double[]{newX, newY};
//            }

            // sort centroids past a certain point
            List<LLResultTypes.DetectorResult> sorted = new LinkedList<>();
            for (LLResultTypes.DetectorResult r : results) {
                double[] position = transform(getCentroid(r.getTargetCorners()));

                // Does the color that it is NOT
                if ((position[0] < 1 || fullRange) &&
                                (r.getClassName().strip().equalsIgnoreCase(color.strip()) ||
                                r.getClassName().strip().equalsIgnoreCase("yellow"))) {
                    sorted.add(r);
                }
            }

            if (!sorted.isEmpty()) {
//                sorted.sort(Comparator.comparingDouble(LLResultTypes.DetectorResult::getTargetArea));
                sorted.sort((a, b) -> {
                            double ay = transform(getCentroid(a.getTargetCorners()))[1];
                            double by = transform(getCentroid(b.getTargetCorners()))[1];

                            return Double.compare(ay, by);
                });

                // Get first element
                int index = 0;
                // Get last element
//                index = sorted.size()-(index + 1);

                outputPosition = transform(getCentroid(sorted.get(index).getTargetCorners()));
                s_color = sorted.get(index).getClassName();
            }
        }

        ArrayList<Object> arr = new ArrayList<>();

        arr.add(outputPosition);
        arr.add(s_color);

        return arr;
    }

    public static double[] transform(double[] centroid) {
        double[] updatedCentroid = applyHomography(H, centroid[0], centroid[1]);

        double newX = updatedCentroid[0]/xTranslate - xOffset;
        double newY = yOffset - updatedCentroid[1]/yTranslate;

        return new double[]{newX, newY};
    }

    public static double[] applyHomography(double[][] H, double x, double y) {
        double xPrime = H[0][0] * x + H[0][1] * y + H[0][2];
        double yPrime = H[1][0] * x + H[1][1] * y + H[1][2];
        double wPrime = H[2][0] * x + H[2][1] * y + H[2][2];

        double xNew = xPrime / wPrime;
        double yNew = yPrime / wPrime;

        return new double[] {xNew, yNew};
    }

    public double[] getCentroid(List<List<Double>> targetCorners) {
        double tX = targetCorners.get(2).get(0);
        tX += targetCorners.get(3).get(0);

        double tY = targetCorners.get(2).get(1);
        tY += targetCorners.get(3).get(1);

        return new double[]{tX/2, tY/2};
    }
}
