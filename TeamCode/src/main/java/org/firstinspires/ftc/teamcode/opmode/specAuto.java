package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;


@Autonomous(name = "specAuto", group = "Into the Deep")
public class specAuto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;
    Pose2d startingPosition = new Pose2d(7.1, -64, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap, startingPosition);
        scheduler = new AutoActionScheduler(this::update);

        robot.arm.grab();
        robot.outtakeSpecAuto();
        robot.lift.setTargetPosition(400);
        robot.intake.intakeUp();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            // ----------------- PRELOAD SPECIMEN  -----------------
            scheduler.addAction(new ParallelAction(
                    //first spec score position
                    robot.drive.actionBuilder(startingPosition)
                            .strafeToLinearHeading(new Vector2d(-2.9, -29), Math.toRadians(90),
                                    new TranslationalVelConstraint(70.0),
                                    new ProfileAccelConstraint(-120, 100))
                            .build(),
                    robot.lift.setTargetPositionAction(1150)));
                            new InstantAction(robot.arm::drop);
            scheduler.run();

            robot.setPose(new Pose2d(-2.9, -29, Math.toRadians(90)));

            // ----------------- SAMPLE 1 -----------------
            scheduler.addAction(new SequentialAction(
                    new InstantAction(robot.arm::drop),
                    robot.lift.setTargetPositionAction(80),
                    new InstantAction(robot.arm::specIntake),
                    robot.drive.actionBuilder(new Pose2d(2.9, -29, Math.toRadians(90)))
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(39, -36), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(46, -17.4), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(57, -6), Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(57, -46), Math.toRadians(-90))
                     .build()
            ));
             scheduler.run();

             robot.relocalizePoll(-62);

            // ----------------- SAMPLE 2 -----------------
            scheduler.addAction(new SequentialAction(
                robot.drive.actionBuilder(robot.drive.pose)
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(58, -17.4), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(72, -15), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(72, -50), Math.toRadians(-90))
                .build())
            );
             scheduler.run();
             robot.relocalizePoll(-62);

            scheduler.addAction(new SequentialAction(
                robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(70, -15), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(80.5, -15), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(80.5, -55), Math.toRadians(-90))
                .build())
            );
            scheduler.run();
            robot.relocalizePoll(-62);

            scheduler.addAction(new SequentialAction(
                    new InstantAction(robot.arm::grab),
                    new SleepAction(0.2),
                    robot.lift.setTargetPositionAction(640),
                    robot.drive.actionBuilder(robot.drive.pose)
                            .setTangent(Math.toRadians(120))
                            .splineToConstantHeading(new Vector2d(-9, -28), Math.toRadians(90))
                            .build()
                    ));
            scheduler.run();

            scheduler.addAction(new SequentialAction(
                    new InstantAction(robot::outtakeSpecAutoVertical),
                    new InstantAction(robot.arm::drop)

            ));
            scheduler.run();

//            scheduler.addAction(new SequentialAction(
//                    new InstantAction(robot.arm::drop),
//                    new InstantAction(robot.arm::autoSpecIntake),
//                    new InstantAction(()-> Lift.targetPosition = 388),
//                    robot.drive.actionBuilder(robot.drive.pose)
//                            .setTangent(Math.toRadians(-90))
//                            .splineToConstantHeading(new Vector2d(39, -64), Math.toRadians(-90))
//                            .build()));
//                    scheduler.run();
//            robot.relocalizePoll(-62);
//
//                    scheduler.addAction(new SequentialAction(
//                    new InstantAction(robot.arm::grab),
//                    new SleepAction(.2),
//                    new InstantAction(()-> Lift.targetPosition = 1125),
//                    new InstantAction(()->robot.outtakeSpecAuto()),
//                    robot.drive.actionBuilder(robot.drive.pose)
//                            .setReversed(true)
//                            .setTangent(90)
//                            .splineToConstantHeading(new Vector2d(-4, -28), Math.toRadians(90))
//                            .build()
//
//            ));
//            scheduler.run();
           // robot.relocalize();


//            ));
        //    scheduler.run();
//            scheduler.addAction(new SequentialAction(
//                    new SleepAction(.5),
//                    new InstantAction(robot.arm::grab),
//                    new SleepAction(.5),
//                    robot.lift.setTargetPositionAction(1700),
//                    new InstantAction(robot::outtakeSpec),
//                    new SleepAction(1),
//                    robot.drive.actionBuilder(new Pose2d(8, -68, Math.toRadians(0)))
//                            .strafeToLinearHeading(new Vector2d(30, 6), Math.toRadians(0))
//                            .strafeToLinearHeading(new Vector2d(42, 6), Math.toRadians(0))
//                            .build(),
//                    new SleepAction(.5),
//                    new InstantAction(robot.arm::drop)
//            ));
//            scheduler.addAction(new ParallelAction(
//                    new InstantAction(robot.arm::specIntake),
//                    robot.specDrop(),
//                    robot.drive.actionBuilder(new Pose2d(42, 6, Math.toRadians(0)))
//                            .strafeToLinearHeading(new Vector2d(9, -38), Math.toRadians(0))
//                            .build()
//            ));
//
//            scheduler.addAction(new SequentialAction(
//                    new SleepAction(.5),
//                    new InstantAction(robot.arm::grab),
//                    new SleepAction(.5),
//                    robot.lift.setTargetPositionAction(1700),
//                    new InstantAction(robot::outtakeSpec),
//                    new SleepAction(1),
//                    robot.drive.actionBuilder(new Pose2d(42, 6, Math.toRadians(0)))
//                            .strafeToLinearHeading(new Vector2d(30, 2), Math.toRadians(0))
//                            .strafeToLinearHeading(new Vector2d(44, 2), Math.toRadians(0))
//                            .build()
//
//            ));
//            scheduler.run();


            sleep(30000);
            return;
        }
    }

    public void update() {
        robot.update();
        telemetry.addData("relocalization", robot.sensors.getSpecimenPosition(robot.drive.pose));
        telemetry.update();

    }
}

