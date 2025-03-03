package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous(name = "03. Sample Auto with Odom", group = "Into the Deep")
public class SampleAutoOdom extends LinearOpMode {

    // TODO: preload position fix, color ejection, timeouts for being stuck, strafing in sub (better pickup)

    public static double intakeDistance1 = 1300;
    public static boolean sampleColorCorrect = false;
    public static double sub = 1100;
    AtomicReference<Double> flip = new AtomicReference<>((double) 1);

    Robot robot;
    AutoActionScheduler scheduler;
    boolean active = true;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    Pose2d preloadBucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));
    Pose2d sample1Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));
    Pose2d sample2Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;
    Pose2d sample3Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));

    Pose2d cycleBucket1 = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;
    Pose2d cycleBucket2 = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;

    Pose2d sample1Pose = new Pose2d(new Vector2d(-48, -42.09), Math.toRadians(92));
    Pose2d sample2Pose = new Pose2d(new Vector2d(-58.92, -42.09), Math.toRadians(92));
    Pose2d sample3Pose = new Pose2d(new Vector2d(-62.2, -42.78), Math.toRadians(112));

    Pose2d subIntake1 = new Pose2d(-22, -12, Math.toRadians(0));
    Pose2d subIntake2 = new Pose2d(-20, -6, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(telemetry, hardwareMap, start, true);
        scheduler = new AutoActionScheduler(this::update);

        Action toBucket = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(preloadBucket.position, preloadBucket.heading)
                .build();

        Action firstSample = robot.drive.actionBuilder(preloadBucket)
                .strafeToLinearHeading(sample1Pose.position, sample1Pose.heading)
                .build();

        Action depositFirst = robot.drive.actionBuilder(sample1Pose)
                .strafeToLinearHeading(sample1Bucket.position, sample1Bucket.heading)
                .build();

        Action secondSample = robot.drive.actionBuilder(sample1Bucket)
                .strafeToLinearHeading(sample2Pose.position, sample2Pose.heading)
                .build();

        Action depositSecond = robot.drive.actionBuilder(sample2Pose)
                .strafeToLinearHeading(sample2Bucket.position, sample2Bucket.heading)
                .build();

        Action thirdSample = robot.drive.actionBuilder(sample2Bucket)
                .strafeToLinearHeading(sample3Pose.position, sample3Pose.heading)
                .build();

        Action depositThird = robot.drive.actionBuilder(sample3Pose)
                .strafeToLinearHeading(sample3Bucket.position, sample3Bucket.heading)
                .build();

        Action moveToSub1 = robot.drive.actionBuilder(sample3Bucket)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(subIntake1, Math.toRadians(0))
                .build();

        Action depositSub1 = robot.drive.actionBuilder(subIntake1)
                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(cycleBucket1, cycleBucket1.heading.toDouble() + Math.toRadians(180))
                .build();

        Action moveToSub2 = robot.drive.actionBuilder(cycleBucket1)
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(subIntake2, Math.toRadians(0))
                .build();

        Action depositSub2 = robot.drive.actionBuilder(subIntake2)
                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(cycleBucket2, cycleBucket2.heading.toDouble() + Math.toRadians(180))
                .build();

        robot.arm.grab();
        robot.intake.intakeUp();
        robot.intake.flickerIn();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            scheduler.addAction(new ParallelAction(
                    bucket(),
                    ActionUtil.Offset(1.3, toBucket, drop())
            ));

            // Cycle 1
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(1, intake(2, intakeDistance1)),
                    ActionUtil.Offset(0.5, firstSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1.4, depositFirst, drop())
            ));

            // Cycle 2
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(1, intake(2, intakeDistance1)),
                    ActionUtil.Offset(0.5, secondSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1.4, depositSecond, drop())
            ));

            // Cycle 3
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(1, intake(2, intakeDistance1)),
                    ActionUtil.Offset(0.5, thirdSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1.4, depositThird, drop())
            ));

            // Cycle sub 1
            scheduler.addAction(
                    ActionUtil.Offset(2.7,
                            ActionUtil.Offset(0.5, moveToSub1, returnLift()),
                            ActionUtil.Offset(0.3, flick(), subIntake(1.5, intakeDistance1)))
            );
            scheduler.run();

            checkForOpColor();

            scheduler.addAction(new ParallelAction(
                    new SequentialAction(
                            transfer(),
                            bucket(true)
                    ),
                    ActionUtil.Offset(2.9, depositSub1, drop())
            ));
            scheduler.addAction(new InstantAction(robot.intake::stopIntake));
            scheduler.addAction(returnLift());
            scheduler.addAction(moveToSub2);
            scheduler.run();

//            // Cycle sub 2
//            scheduler.addAction(
//                    ActionUtil.Offset(2.2,
//                            ActionUtil.Offset(0.5, moveToSub2, returnLift()),
//                            ActionUtil.Offset(0.3, flick(), subIntake(1.5, intakeDistance1)))
//            );

            //checkForOpColor();
//
//            scheduler.addAction(new ParallelAction(
//                    new SequentialAction(
//                            transfer(),
//                            bucket(true)
//                    ),
//                    ActionUtil.Offset(2.3, depositSub2, drop())
//            ));
//            scheduler.run();
            scheduler.addAction(robot.endAuto( this, telemetry, 30));
            scheduler.run();
        }
    }

    public void checkForOpColor() {
        sampleColorCorrect = robot.intake.isRightColor();
        while (!sampleColorCorrect && active) {
            scheduler.addAction(
                    new SequentialAction(
                    new ParallelAction(
                            robot.eject(),
                            robot.intake.setTargetPosition(0),
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeTo(new Vector2d(robot.drive.pose.position.x, robot.drive.pose.position.y+6))
                                    .build()),
                    subIntake(2, intakeDistance1),
                    new InstantAction(()->sampleColorCorrect = robot.intake.isRightColor())
            ));
            scheduler.run();
        }
    }

    public Action intake(double timeout, double distance) {
        return new SequentialAction(
                new InstantAction(()-> flip.set(0.0)),
                new ActionUtil.RunnableTimedAction(timeout, ()-> {
                    flip.getAndSet(new Double((double) (flip.get() + 1)));
                    robot.intake.startIntake();
                    Intake.PID_ENABLED = false;
                    robot.intake.extension.setPower(1);
                    //robot.intake.intakeDown();

                    if (robot.intake.extension.getCurrentPosition() > 1100) {
                        if (flip.get() % 10 == 0) robot.intake.intakeDown();
                        else robot.intake.intakeUp();
                    } else if (flip.get() % 10 != 0) {
                        robot.intake.intakeDown();
                    }

                    if(robot.intake.hasSample()) return false;

                    return !robot.intake.hasSample();
                }),
                new InstantAction(()-> {
                    Intake.targetPosition = distance;
                    Intake.PID_ENABLED = true;
                }));
    }

    public Action subIntake(double timeout, double distance) {
        return new SequentialAction(
                new InstantAction(()-> flip.set(0.0)),
                new ActionUtil.RunnableTimedAction(timeout, ()-> {
                    Intake.PID_ENABLED = false;
                    robot.intake.extension.setPower(1);

                    //Intake.targetPosition = distance;
                    robot.intake.spin.setPower(1);

                    if (robot.intake.extension.getCurrentPosition() > 50){
                        robot.intake.intakeDown();
                    }

                    if (robot.intake.extension.getCurrentPosition() > 1100) {
                        if (flip.get() % 10 == 0) robot.intake.intakeDown();
                        else robot.intake.intakeUp();
                    } else if (flip.get() % 10 != 0) {
                        robot.intake.intakeDown();
                    }
                    sampleColorCorrect = robot.intake.isRightColor();

//                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.1, 0.2), 0));

                    if(robot.intake.hasSample()) return false;

                    return !robot.intake.hasSample();
                }),
                new InstantAction(()-> {
                    Intake.targetPosition = distance;
                    Intake.PID_ENABLED = true;
                    sampleColorCorrect = robot.intake.isRightColor();
                }));
    }

    public Action transfer() {
        return new SequentialAction(new InstantAction(()-> robot.TRANSFER_STATE = Robot.tranfserState.IDLE),
                new ActionUtil.RunnableAction(()-> {
                    //robot.intake.spin.setPower(0);
                    robot.transferFSM();

                    if (!robot.intake.hasSample()) {
                        Intake.targetPosition = 0;
                        return false;
                    }

                    return !robot.TRANSFER_STATE.equals(Robot.tranfserState.DONE);
                }));
    }

    public Action drop() {
        return new ParallelAction(
                robot.sampleDrop()
        );
    }

    public Action flick() {
        return new SequentialAction(
                new InstantAction(robot.intake::flickerOut),
                new SleepAction(0.4),
                new InstantAction(robot.intake::flickerIn)
        );
    }

    public Action returnLift() {
        return new InstantAction(()-> {
            Lift.targetPosition = 0;
            robot.arm.intakePrimePosition();
        });
    }

    public Action bucket() {
        return bucket(false);
    }

    public Action bucket(boolean cycle) {
        return new ParallelAction(
                new InstantAction(()-> {
                    robot.intake.reverseIntake();
                    //robot.intake.stopIntake();
                    robot.outtakeBucket();
                    if (cycle) robot.arm.setPivot(Arm.PIVOT_ARM_PRIME_AUTO);
                })
        );
    }

    public void update() {
        robot.update();

        if (robot.intake.hasSample()) active = false;
        else active = true;
//
//        telemetry.addData("scheudluer", scheduler.isBusy());
//        telemetry.addData("current action", scheduler.currentAction().getClass().getName());
//        telemetry.addData("Color correct", sampleColorCorrect);
//        telemetry.update();
    }
}
