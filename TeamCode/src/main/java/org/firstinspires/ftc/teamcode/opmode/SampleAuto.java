package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Config
@Autonomous
public class SampleAuto extends LinearOpMode {

    // TODO: preload position fix, color ejection, timeouts for being stuck, strafing in sub (better pickup)

    public static double intakeDistance1 = 1300;
    public static double sub = 1100;
    Robot robot;
    AutoActionScheduler scheduler;
    double startTime = System.nanoTime() / 1e9;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    Pose2d preloadBucket = new Pose2d(new Vector2d(-64.86, -56.32), Math.toRadians(58));
    Pose2d sample1Bucket = new Pose2d(new Vector2d(-65.11, -57.17), Math.toRadians(49));
    Pose2d sample2Bucket = new Pose2d(new Vector2d(-65.11, -57.17), Math.toRadians(49));
    Pose2d sample3Bucket = new Pose2d(new Vector2d(-65.11, -57.17), Math.toRadians(49));

    Pose2d cycleBucket1 = new Pose2d(new Vector2d(-62.15, -58.94), Math.toRadians(47));
    Pose2d cycleBucket2 = new Pose2d(new Vector2d(-56.37, -62.81), Math.toRadians(47));

    Pose2d sample1Pose = new Pose2d(new Vector2d(-51.6, -38.35), Math.toRadians(90));
    Pose2d sample2Pose = new Pose2d(new Vector2d(-65.5, -38.35), Math.toRadians(92  ));
    Pose2d sample3Pose = new Pose2d(new Vector2d(-68.76, -39.47), Math.toRadians(118.63));
    //Pose2d sample3Pose = new Pose2d(new Vector2d(-67.27, -45.7), Math.toRadians(115.3));

    Pose2d subIntake1 = new Pose2d(-25, 0, Math.toRadians(0));
    Pose2d subIntake2 = new Pose2d(-20, 6, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(telemetry, hardwareMap, start);
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
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            scheduler.addAction(new ParallelAction(
                    bucket(),
                    ActionUtil.Offset(1.1, toBucket, drop())
            ));

            // Cycle 1
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(0.5, intake(2,0.5, intakeDistance1)),
                    ActionUtil.Offset(0.5, firstSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1, depositFirst, drop())
            ));

            // Cycle 2
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(1, intake(2,0.5, intakeDistance1)),
                    ActionUtil.Offset(0.5, secondSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1, depositSecond, drop())
            ));

            // Cycle 3
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(1, intake(2,0.5, intakeDistance1)),
                    ActionUtil.Offset(0.5, thirdSample, returnLift())
            ));
            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(1, depositThird, drop())
            ));

            // Cycle sub 1
            scheduler.addAction(new ParallelAction(
                    //ActionUtil.Delay(0.5, intake(2,0.5, intakeDistance1)),
                    ActionUtil.Offset(0.5, moveToSub1, returnLift()) // TODO: make this and the following action in one add action, and make the flick flick sooner
            ));
            scheduler.addAction(new SequentialAction(
                    ActionUtil.Offset(0.3, flick(), subIntake(2, 0.5, intakeDistance1))
            ));

            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(2.3, depositSub1, drop())
            ));

            // Cycle sub 2
            scheduler.addAction(new ParallelAction(
                    //ActionUtil.Delay(0.5, intake(2,0.5, intakeDistance1)),
                    ActionUtil.Offset(0.5, moveToSub2, returnLift()) // TODO: make this and the following action in one add action, and make the flick flick sooner
            ));
            scheduler.addAction(new SequentialAction(
                    ActionUtil.Offset(0.3, flick(), subIntake(2, 0.5, intakeDistance1))
            ));

            scheduler.addAction(new SequentialAction(
                    new SequentialAction(
                            transfer(),
                            bucket()
                    ),
                    ActionUtil.Offset(2.3, depositSub2, drop())
            ));

            scheduler.run();
            scheduler.addAction(robot.endAuto(telemetry, 30));
            scheduler.run();
        }
    }

    public Action intake(double longTimeout, double shortTimeout, double distance) {

        return new SequentialAction(
                new InstantAction(()-> {
                    startTime = System.nanoTime() / 1e9;
                }),
                new ActionUtil.RunnableAction(()-> {
                    Intake.PID_ENABLED = true;
                    Intake.targetPosition = distance;
                    robot.intake.spin.setPower(1);
                    robot.intake.intakeDown();

                    if (robot.intake.extension.getCurrentPosition() > 1100) {
                        robot.intake.intakeUp();
                    } else {
                        //robot.intake.intakeDown();
                    }

                    if(robot.intake.hasSample()) return false;

                    return (startTime + longTimeout < (System.nanoTime() / 1e9) || !robot.intake.hasSample());
                }
        ));
    }

    public Action subIntake(double longTimeout, double shortTimeout, double distance) {

        return new SequentialAction(
                new InstantAction(()-> {
                    startTime = System.nanoTime() / 1e9;
                }),
                new ActionUtil.RunnableAction(()-> {
                    Intake.PID_ENABLED = true;
                    Intake.targetPosition = distance;
                    robot.intake.spin.setPower(1);
                    robot.intake.intakeDown();

                    if (robot.intake.extension.getCurrentPosition() > 1100) {
                        robot.intake.intakeUp();
                    } else {
                        //robot.intake.intakeDown();
                    }

                    if(robot.intake.hasSample()) return false;

                    return (startTime + longTimeout < (System.nanoTime() / 1e9) || !robot.intake.hasSample());
                }
                ));
    }

    public Action transfer() {
        return new ActionUtil.RunnableAction(()-> {
            robot.intake.spin.setPower(0);
            robot.transferFSM();

            return !robot.TRANSFER_STATE.equals(Robot.tranfserState.TO_POSITION);
        });
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
        return new ParallelAction(
                new InstantAction(()-> {
                    robot.intake.stopIntake();
                    robot.outtakeBucket();
                })
        );
    }

    public void update() {
        robot.update();
        //telemetry.update();
    }
}
