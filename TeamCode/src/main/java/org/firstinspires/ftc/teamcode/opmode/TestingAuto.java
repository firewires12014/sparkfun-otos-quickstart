package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;


@Autonomous(name = "00. Test Auto", group = "Into the Deep")
public class TestingAuto extends LinearOpMode {
    Auto auto;
    AutoActionScheduler scheduler;
    Pose2d startingPosition = new Pose2d(7.1, -64, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        auto = new Auto(telemetry, hardwareMap, startingPosition);
        scheduler = new AutoActionScheduler(this::update);

        auto.arm.grab();
        auto.outtakeSpecAuto();
        auto.lift.setTargetPosition(400);
        auto.intake.intakeUp();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            // ----------------- PRELOAD SPECIMEN  -----------------
            scheduler.addAction(new ParallelAction(
                new InstantAction(auto.arm::drop)
            ));
            scheduler.run();
        }
    }

    public void update() {
        auto.update();
        telemetry.addData("relocalization", auto.sensors.getSpecimenPosition(auto.drive.pose));
        telemetry.update();

    }
}

