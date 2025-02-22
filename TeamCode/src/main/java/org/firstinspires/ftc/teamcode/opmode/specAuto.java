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

import org.firstinspires.ftc.teamcode.subsystems.Arm;
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
        robot.outtakeSpec();
        robot.lift.setTargetPosition(400);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            scheduler.addAction(
            //first spec score position
                    robot.drive.actionBuilder(startingPosition)
                            .strafeToLinearHeading(new Vector2d(35, 10), Math.toRadians(0),
                                    new TranslationalVelConstraint(70.0),
                                    new ProfileAccelConstraint(-120, 100))
                            .build());
            //raise lift to specimen position
            robot.lift.setTargetPositionAction(1700);
            scheduler.run();

            sleep(500);

            //raise lift a little more to cl
            // ip specimen
            scheduler.addAction(
            robot.lift.setTargetPositionAction(1900));

            scheduler.run();
            sleep(500);
            //drop specimen
            robot.arm.drop();
            sleep(200);

            scheduler.addAction(new ParallelAction(
                    //move arm/wrist to position to pick up specimen from wall
                    new InstantAction(robot.arm::specIntake),
                    //move lift to height for specimen pickup and open claw
                    robot.specDrop(),

                    robot.drive.actionBuilder(new Pose2d(35, 10, Math.toRadians(0)))
                            .setReversed(true)
                            .setTangent(Math.toRadians(80))
                    //drive away from submersible
                            .splineToConstantHeading(new Vector2d(28, -35), Math.toRadians(0))
                            .setTangent(Math.toRadians(0))
                    //spline to push first sample in
                            .splineToConstantHeading(new Vector2d(58, -47), Math.toRadians(-90))
                    //push first sample in
                            .strafeToLinearHeading(new Vector2d(18, -47), Math.toRadians(0))
//                                    new TranslationalVelConstraint(200.0),
//                                    new ProfileAccelConstraint(-120, 120))
                            .setTangent(0)
                    //spline to second sample
                            .splineToConstantHeading(new Vector2d(58, -60), Math.toRadians(-90))
                    //push second sample
                            .strafeToLinearHeading(new Vector2d(18, -60), Math.toRadians(0))
//                                    new TranslationalVelConstraint(200.0),
//                                    new ProfileAccelConstraint(-120, 120))
                            .setTangent(0)
                    //spline to third sample
                            .splineToConstantHeading(new Vector2d(58, -69), Math.toRadians(-90))
                    // push third sample and go to position to pick up specimen
                            .strafeToLinearHeading(new Vector2d(8, -68), Math.toRadians(0))
//                                    new TranslationalVelConstraint(200.0),
//                                    new ProfileAccelConstraint(-120, 120))
                            .build()
            ));
            scheduler.addAction(new SequentialAction(
                    new SleepAction(.5),
                    new InstantAction(robot.arm::grab),
                    new SleepAction(.5),
                    robot.lift.setTargetPositionAction(1700),
                    new InstantAction(robot::outtakeSpec),
                    new SleepAction(1),
                    robot.drive.actionBuilder(new Pose2d(8, -68, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(30, 6), Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(42, 6), Math.toRadians(0))
                            .build(),
                    new SleepAction(.5),
                    new InstantAction(robot.arm::drop)
            ));
            scheduler.addAction(new ParallelAction(
                    new InstantAction(robot.arm::specIntake),
                    robot.specDrop(),
                    robot.drive.actionBuilder(new Pose2d(42, 6, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(9, -38), Math.toRadians(0))
                            .build()
            ));

            scheduler.addAction(new SequentialAction(
                    new SleepAction(.5),
                    new InstantAction(robot.arm::grab),
                    new SleepAction(.5),
                    robot.lift.setTargetPositionAction(1700),
                    new InstantAction(robot::outtakeSpec),
                    new SleepAction(1),
                    robot.drive.actionBuilder(new Pose2d(42, 6, Math.toRadians(0)))
                            .strafeToLinearHeading(new Vector2d(30, 2), Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(44, 2), Math.toRadians(0))
                            .build()

            ));
            scheduler.run();


            sleep(30000);
            return;
        }
    }

    public void update() {
        robot.update();
        telemetry.update();
    }
}

