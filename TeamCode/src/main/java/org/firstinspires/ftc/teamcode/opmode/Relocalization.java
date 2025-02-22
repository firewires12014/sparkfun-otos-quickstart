package org.firstinspires.ftc.teamcode.opmode;


import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

@TeleOp
@Config
public class Relocalization extends LinearOpMode {
    public static double Q=.15, R=3, N=3;
    public static double offestX = 0;
    public static double offsetY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry (telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        Sensors sensor = new Sensors(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        KalmanFilter filter = new KalmanFilter(Q,R, (int) N);
        while (opModeIsActive()) {
           // filter = new KalmanFilter(Q,R, (int) N);
            TelemetryPacket packet = new TelemetryPacket();


            telemetry.addData("back", sensor.getBack());
            telemetry.addData("right", sensor.getRight());
            double x = 72-sensor.getBack() + Sensors.backOffset[1];
            double y = 72-sensor.getRight() - Sensors.rightOffset[0];
            Canvas c = packet.fieldOverlay();
            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, new Pose2d(x, y, Math.toRadians(-90)));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addData("position", (x + offestX) + "\t" + (y + offsetY));
            drive.pose = new Pose2d(x, y, Math.toRadians(-90));
            telemetry.addData("kalmanBack", filter.estimate(sensor.getBack()));
            telemetry.update();

        }

    }
}
