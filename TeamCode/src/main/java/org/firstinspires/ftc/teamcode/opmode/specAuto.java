package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;


@Autonomous(name="specAuto", group="Into the Deep")
public class specAuto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;
    Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        scheduler = new AutoActionScheduler(this::update);

        robot.arm.grab();
        robot.outtakeSpec();

        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            scheduler.addAction(
                    robot.drive.actionBuilder(startingPosition)
                    .strafeToLinearHeading(new Vector2d(35, 2.5), Math.toRadians(0),
                            new TranslationalVelConstraint(70.0))
                    .build());
            scheduler.run();
            sleep(500);
            robot.specScore();
            scheduler.run();
           // sleep(200);
            robot.arm.drop();
            sleep(100);

            scheduler.addAction(new ParallelAction(
                    new InstantAction(robot.arm::specIntake),
                    robot.specDrop(),
                    robot.drive.actionBuilder(new Pose2d(35, 2.5, Math.toRadians(0)))
                            .setReversed(true)
                            .setTangent(Math.toRadians(80))
                            .splineToConstantHeading(new Vector2d(28, -35), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(56, -47), Math.toRadians(90))
                            .build()));
            scheduler.run();


            sleep(30000);
            return;
        }
    }

    public void update(){
        robot.update();
        telemetry.update();
    }
}

