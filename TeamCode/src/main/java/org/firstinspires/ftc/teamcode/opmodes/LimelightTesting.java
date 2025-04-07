package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp
//@Disabled
public class LimelightTesting extends LinearOpMode {
    public static double xTranslate = 42;
    public static double yTranslate = 24;
    public static double yOffset = 25;
    public static double xOffset = 2.6;

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        // Any pre start init shi
        double[][] H = new double[][] {
                {-1.97539577e+01, -2.15267488e+01,  3.83024175e+03},
                {-3.50811062e+00, -6.66541018e+01,  6.20935580e+03},
                {-6.93140729e-03, -7.92889642e-02,  1.00000000e+00}
        };

        waitForStart();
        resetRuntime();
        limelight.start();
        prevLoop = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
            // Insert actual code
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                List<LLResultTypes.DetectorResult> results = result.getDetectorResults();

                for(LLResultTypes.DetectorResult dr : results) {
                    double[] centroid = getCentroid(dr.getTargetCorners());

                    double[] updatedCentroid = applyHomography(H, centroid[0], centroid[1]);

                    double newX = updatedCentroid[0]/xTranslate - xOffset;
                    double newY = yOffset - updatedCentroid[1]/yTranslate;


                    telemetry.addData("name", dr.getClassName());
                    telemetry.addData("area", dr.getTargetArea());
                    telemetry.addData("centroid", Arrays.toString(centroid));
                    telemetry.addData("homo centroid", Arrays.toString(updatedCentroid));
                    telemetry.addData("translated centroid", String.format("X: %.2f \t Y: %.2f", newX, newY));
                }
            }

            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
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

    public void loopTimeMeasurement(Telemetry telemetry) {
        double currTime = System.nanoTime() / 1e9;
        double delta = currTime - prevLoop;
        telemetry.addData("loop time", delta);
        telemetry.addData("hz", 1 / delta);
        telemetry.update();
        prevLoop = System.nanoTime() / 1e9;
    }
}
