package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
        robot.lift.setTargetPositionAction(400);

        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            scheduler.addAction(
                    robot.drive.actionBuilder(startingPosition)
                    .strafeToLinearHeading(new Vector2d(35, 2.5), Math.toRadians(0),
                        new TranslationalVelConstraint(200.0),
                        new ProfileAccelConstraint(-120, 100))
                    .build());
            scheduler.run();
            sleep(250);
            robot.specScore();
            //i dont think the lift thing does anything
            robot.lift.setTargetPositionAction(1700);
            sleep(80);
            scheduler.run();
            robot.arm.drop();
            sleep(200);

            scheduler.addAction(new ParallelAction(
                    new InstantAction(robot.arm::specIntake),
                    robot.specDrop(),
                    robot.drive.actionBuilder(new Pose2d(35, 2.5, Math.toRadians(0)))
                            .setReversed(true)
                            .setTangent(Math.toRadians(80))
                            .splineToConstantHeading(new Vector2d(28, -35), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(56, -47), Math.toRadians(-90))
                            .strafeToLinearHeading(new Vector2d(17, -47), Math.toRadians(0),
                                new TranslationalVelConstraint(200.0),
                                new ProfileAccelConstraint(-120, 120))
                            .setTangent(0)
                            .splineToConstantHeading(new Vector2d(61, -60), Math.toRadians(-90))
                            .strafeToLinearHeading(new Vector2d(17, -60), Math.toRadians(0),
                                new TranslationalVelConstraint(200.0),
                                new ProfileAccelConstraint(-120, 120))
                            .setTangent(0)
                            .splineToConstantHeading(new Vector2d(61, -69), Math.toRadians(-90))
                            .strafeToLinearHeading(new Vector2d(17, -69), Math.toRadians(0),
                                new TranslationalVelConstraint(200.0),
                                new ProfileAccelConstraint(-120, 120))
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

