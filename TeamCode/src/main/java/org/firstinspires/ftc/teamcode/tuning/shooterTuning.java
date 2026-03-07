package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp
@Config
public class shooterTuning extends LinearOpMode {

    public static double targetVelocity = 0;
    public static double hoodPosition = 0;
    public static double targetX = -67;
    public static double targetY = 67;

    @Override
    public void runOpMode() throws InterruptedException {

        Shooter shoot = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Drive drive = new Drive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            shoot.setVelocity(targetVelocity);
            if (gamepad1.left_trigger > 0) {
                intake.in();
                transfer.run();
            }
            else if (gamepad1.right_trigger > 0) {
                intake.out();
                transfer.reverse();
            }
            else {
                intake.stop();
                transfer.stop();
            }
            if (gamepad1.circle) {
             transfer.triggerOpen();
            }
            if (gamepad1.cross) {
                transfer.triggerClose();
            }

            if (gamepad1.triangle) {
                hood.hood.setPosition(hoodPosition);
            }

            drive.update();
            Pose2d pose = drive.getPose();

            double distance = Math.sqrt(Math.pow(targetX - pose.position.x, 2)+Math.pow(targetY - pose.position.y, 2));

            //transfer.run();
            shoot.pidUpdate();

            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("actualVelocity", shoot.shooter.getVelocity());
            telemetry.addData("0", 0);
            telemetry.addData("distance", distance);
            telemetry.update();


        }
    }
}
