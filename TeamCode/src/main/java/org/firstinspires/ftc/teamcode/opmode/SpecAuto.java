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

import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;


@Autonomous(name = "01. Specimen Auto", group = "Into the Deep")
public class SpecAuto extends LinearOpMode {
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
                    //first spec score position
                    auto.drive.actionBuilder(startingPosition)
                            .strafeToLinearHeading(new Vector2d(-2.9, -29), Math.toRadians(90),
                                    new TranslationalVelConstraint(70.0),
                                    new ProfileAccelConstraint(-120, 100))
                            .build(),
                    auto.lift.setTargetPositionAction(1150)));
                            new InstantAction(auto.arm::drop);
            scheduler.run();

            auto.setPose(new Pose2d(-2.9, -29, Math.toRadians(90)));

            // ----------------- SAMPLE 1 -----------------
            scheduler.addAction(new SequentialAction(
                    new InstantAction(auto.arm::drop),
                    auto.lift.setTargetPositionAction(80),
                    new InstantAction(auto.arm::specIntake),
                    auto.drive.actionBuilder(new Pose2d(2.9, -29, Math.toRadians(90)))
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(39, -36), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(46, -17.4), Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(57, -6), Math.toRadians(-90))
                            .splineToConstantHeading(new Vector2d(57, -46), Math.toRadians(-90))
                     .build()
            ));
             scheduler.run();

            auto.relocalizePoll(-62);

            // ----------------- SAMPLE 2 -----------------
            scheduler.addAction(new SequentialAction(
                    auto.drive.actionBuilder(auto.drive.pose)
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(58, -17.4), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(72, -15), Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(72, -50), Math.toRadians(-90))
                .build())
            );
             scheduler.run();
            auto.relocalizePoll(-62);

            scheduler.addAction(new SequentialAction(
                    auto.drive.actionBuilder(auto.drive.pose)
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(70, -15), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(80.5, -15), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(80.5, -55), Math.toRadians(-90))
                .build())
            );
            scheduler.run();
            auto.relocalizePoll(-62);

            scheduler.addAction(new SequentialAction(
                    new InstantAction(auto.arm::grab),
                    new SleepAction(0.2),
                    auto.lift.setTargetPositionAction(640),
                    auto.drive.actionBuilder(auto.drive.pose)
                            .setTangent(Math.toRadians(120))
                            .splineToConstantHeading(new Vector2d(-9, -28), Math.toRadians(90))
                            .build()
                    ));
            scheduler.run();

            scheduler.addAction(new SequentialAction(
                    new InstantAction(auto::outtakeSpecAutoVertical),
                    new InstantAction(auto.arm::drop)

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
        auto.update();
        telemetry.addData("relocalization", auto.sensors.getSpecimenPosition(auto.drive.pose));
        telemetry.update();

    }
}

