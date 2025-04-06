package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
@Config
// TODO: Fix all this (TEST ORIGINAL WITH THE PINPOINT REST)
//       - Have outputs based on ticks (DONE)
//       - Ignore Arc-Length Formula | only use Tangential Velocity Formula (DONE)
//       - Save all Angular and Encoder velo as a point (Angular, Encoder), write the array to a file (DONE)
//       - Use ggplot nonsense to get a proper linear regression to get offset distance (uh do when i have the csv if needed)

public class OffsetTuning extends LinearOpMode {
    // Config Variables
    public static boolean useIMU = false;
    public static double incrementPower = 0.1;
    public static double startPower = 0.1;
    public static double endPower = 0.95;
    public static double spinDirection = 1;
    private double currentPower = startPower;
    private String fileOutputName = "output";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
        IMU imu = hardwareMap.get(IMU.class, "imu");

        ArrayList<Double> parPoints = new ArrayList<>(), perpPoints = new ArrayList<>(), angularPoints = new ArrayList<>();
        ElapsedTime speedTimer = new ElapsedTime();

        while (opModeInInit() && !isStopRequested()) {
            if (useIMU) {
                telemetry.addData("IMU selected", "DEFAULT IMU");
            } else telemetry.addData("IMU selected", "PINPOINT IMU");
            telemetry.addLine("If the following values are not close to zero reset the pinpoint");
            telemetry.addLine("To reset the pinpoint unplug the i2c cable wait a second then replug it");
            telemetry.addData("Encoders", "Par: %d, Perp: %d", localizer.driver.getEncoderX(), localizer.driver.getEncoderY());
            telemetry.update();
        }

        waitForStart();
        telemetry.clear();
        speedTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            while (currentPower < endPower) {
                /**
                 *  Dead wheel locations / labels
                 *                 ^
                 *   <- y (perp)   |
                 *                 x (par)
                 *
                 */

                // update pinpoint each loop, will technically tank loop times :(
                localizer.driver.update();

                // Increment Drive Power
                currentPower = Math.min(startPower * speedTimer.seconds(), endPower);

                // Set Drive power
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                0
                        ),
                        currentPower * spinDirection
                ));

                double parVelo = localizer.driver.getVelX(); // Velocity for the X encoder
                double perpVelo = localizer.driver.getPosY(); // Velocity for the Y encoder

                double angularVelocity;
                if (useIMU) angularVelocity = Math.toRadians(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate); // Bug in 10.1 and below, has since been fixed in 10.2
                else angularVelocity = localizer.driver.getHeadingVelocity();

                angularPoints.add(angularVelocity);
                parPoints.add(parVelo);
                perpPoints.add(perpVelo);

                telemetry.addLine("Velocity Information");
                telemetry.addData("Current Power", currentPower);
                telemetry.addLine("NOTE: Angular Velocity MUST be positive, if not flip the sign for spinDirection in Config");
                telemetry.addData("Par Encoder", parVelo);
                telemetry.addData("Perp Encoder", perpVelo);
                telemetry.addData("Angular Velo", angularVelocity);
                telemetry.update();
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // Stop Spinning

            telemetry.addLine("Rotation Done!");
            telemetry.addData("Regression Size", angularPoints.size());
            telemetry.update();

            try (FileWriter writer = new FileWriter(AppUtil.ROOT_FOLDER.toString() + "/CustomOffsetTuner/" + fileOutputName + System.currentTimeMillis() + ".csv")) {
                writer.write("Angular Velocity,Par Velocity,Perp Velocity\n");
                for (int i = 0; i < angularPoints.size(); i++) {
                    writer.write(angularPoints.get(i) + "," + parPoints.get(i) + "," + perpPoints.get(i) + "\n");
                }

                telemetry.addLine("CSV file written successfully.");
            } catch (IOException e) {
                // e.printStackTrace();
                telemetry.addData("Error writing csv file", e.getClass().getName());
            }

            double parYTicks = olsLinearRegression(angularPoints, parPoints);
            double perpXTicks = olsLinearRegression(angularPoints, perpPoints);

            telemetry.addLine("Basic Linear Regression: Outliers are NOT removed these could be wrong");
            telemetry.addLine("Disregard the sign for these values, parYTicks: -, perpXTicks: +");
            telemetry.addData("Regression Size", angularPoints.size());
            telemetry.addData("parYTicks", parYTicks);
            telemetry.addData("perpXTicks", perpXTicks);
            telemetry.update();

            sleep((long) 1e24); // hang for a hella long time
        }

    }

    // Fits through the origin, since velo should start at 0
    public static double olsLinearRegression(ArrayList<Double> x, ArrayList<Double> y) {
        int n = x.size();
        if (n != y.size() || n == 0) {
            throw new IllegalArgumentException("Lists must be of equal non-zero length.");
        }

        double sumXY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumXY += x.get(i) * y.get(i);
            sumX2 += x.get(i) * x.get(i);
        }

        // Slope
        return sumXY / sumX2;
    }
}
