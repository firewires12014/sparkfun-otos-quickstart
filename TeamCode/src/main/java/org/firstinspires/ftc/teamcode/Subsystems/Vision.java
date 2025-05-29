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
    public static double falloffPower = 2;
    public static double pBoxWeight = 2;
    public static double[] distanceCenter = new double[]{0, 5};
    public static double[] pBoxCenter = new double[]{0, 2};
    public static double[] pBoxDim = new double[]{2, 2};

    public static double xWeight = 0.06;
    public static double yWeight = 0.14;

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

        double[] outputPosition = new double[]{-1, 2}; // default position

        String s_color = "?";

        if (result != null) {
            List<LLResultTypes.DetectorResult> results = result.getDetectorResults();

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
                sorted.sort((a, b) -> { // still doesn't deprioritize area around blue samples but it should be fine
                    double[] axy = transform(getCentroid(a.getTargetCorners()));
                    double[] bxy = transform(getCentroid(b.getTargetCorners()));

                    // Circular Shape
                    double aDist = Math.sqrt(Math.pow(Math.abs(axy[0] - distanceCenter[0]) * xWeight, 2) + Math.abs(axy[1] - distanceCenter[1]) * yWeight);
                    double bDist = Math.sqrt(Math.pow(Math.abs(bxy[0] - distanceCenter[0]) * xWeight, 2) + Math.abs(bxy[1] - distanceCenter[1]) * yWeight);

                    double aWeight = Math.pow(aDist, falloffPower);
                    double bWeight = Math.pow(bDist, falloffPower);

                    double left   = pBoxCenter[0] - pBoxDim[0] / 2;
                    double right  = pBoxCenter[0] + pBoxDim[0] / 2;
                    double top    = pBoxCenter[1] + pBoxDim[1] / 2;
                    double bottom = pBoxCenter[1] - pBoxDim[1] / 2;

                    //y > bottom and y < top and x > left and x < right

                    if (axy[1] > bottom && axy[1] < top && axy[0] > left && axy[0] < right) {
                        aWeight += pBoxWeight;
                    }

                    if (bxy[1] > bottom && bxy[1] < top && bxy[0] > left && bxy[0] < right) {
                        bWeight += pBoxWeight;
                    }

                    double ay = Math.abs(axy[1]);

                    double by = Math.abs(bxy[1]);

                    // a, b (lowest to greatest), b, a (greatest to lowest)
                    return Double.compare(ay, by); // currently index = 0 (best selection)
                });

                // Get first element
                int index = 0;

                outputPosition = transform(getCentroid(sorted.get(index).getTargetCorners()));
                s_color = sorted.get(index).getClassName();
            }
        }

        ArrayList<Object> arr = new ArrayList<>();

        arr.add(outputPosition);
        arr.add(s_color);
        arr.add(pBoxWeight);

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
        double tX = targetCorners.get(0).get(0);
        tX += targetCorners.get(2).get(0);

        double tY = targetCorners.get(0).get(1);
        tY += targetCorners.get(2).get(1);

        return new double[]{tX/2, tY/2};
    }

    public double[] getBottomCentroid(List<List<Double>> targetCorners) {
        double tX = targetCorners.get(2).get(0);
        tX += targetCorners.get(3).get(0);

        double tY = targetCorners.get(2).get(1);
        tY += targetCorners.get(3).get(1);

        return new double[]{tX/2, tY/2};
    }
}
