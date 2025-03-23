package org.firstinspires.ftc.teamcode.opmodes;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Config
@Autonomous
//@Disabled
public class SampleAuto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    Pose2d preloadBucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample1Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample2Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample3Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));

    Pose2d sample1Pose = new Pose2d(new Vector2d(-56.33, -48.87), Math.toRadians(73.39));
    Pose2d sample2Pose = new Pose2d(new Vector2d(-58.8, -47), Math.toRadians(95));
    Pose2d sample3Pose = new Pose2d(new Vector2d(-62.15, -48.0), Math.toRadians(115));

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
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

        // Any pre start init shi
        robot.farm.close();
        robot.intake.intakeUp();
        waitForStart();
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
            // Insert actual code
            scheduler.addAction(new ParallelAction(
                    bucket(),
                    ActionUtil.Offset(1.3, toBucket, dropFard())
            ));

            // Cycle 1
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(0.2, returnLift()),
                    firstSample,
                    ActionUtil.Delay(0.5, intake(1, 520))
            ));
            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.7, depositFirst, dropFard())
            ));

            // Cycle 2
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(0.2, returnLift()),
                    secondSample,
                    ActionUtil.Delay(0.5, intake(1, 520))
            ));
            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.7, depositSecond, dropFard())
            ));

            // Cycle 3
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(0.2, returnLift()),
                    thirdSample,
                    ActionUtil.Delay(0.5, intake(1, 520))
            ));
            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.7, depositThird, dropFard())
            ));
            scheduler.run();

            // End 4 sample
            scheduler.addAction(ActionUtil.Delay(0.2, returnLift()));

            scheduler.addAction(robot.endAuto( this, telemetry, 30));
            scheduler.run();

            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }

    public Action drop(double timeout, double distance) {
        return new SequentialAction(
                new ActionUtil.RunnableTimedAction(timeout, ()-> robot.inRangeOfBucket()),
                new InstantAction(robot.farm::drop)
        );
    }

    public Action drop(double timeout) {
        return drop(timeout, 220);
    }

    public Action drop() {
        return drop(1, 220);
    }

    public Action dropFard() {
        return new InstantAction(()-> {
            robot.farm.drop();
            robot.intake.intakeHorizontal();
        });
    }

    public Action bucket() {
        return new InstantAction(()-> {
                    robot.farm.setBucketScore();
                }
        );
    }

    public Action intake(double timeout, double distance) {
        return new SequentialAction(
                new ActionUtil.RunnableTimedAction(timeout, ()-> {
                    robot.intake.startIntake();
                    Intake.PID_ENABLED = false;
                    robot.intake.extension.setPower(1);
                    robot.intake.intakeDown();

                    if(robot.hasSample()) return false;

                    return !robot.hasSample();
                }),
                new InstantAction(()-> {
                    Intake.targetPosition = distance;
                    Intake.PID_ENABLED = true;
                }));
    }

    public Action returnLift() {
        return new InstantAction(robot.farm::setTransfer);
    }

    public Action transfer() {
       return  new SequentialAction(
                new InstantAction(()-> {
                    robot.intake.startIntake();
                    Intake.PID_ENABLED = false;
                    robot.intake.extension.setPower(-1);
                    robot.intake.intakeUp();
                    robot.farm.setTransfer();
                }),
                new ActionUtil.RunnableAction(()-> robot.intake.extension.getCurrentPosition() > 50),
                new SleepAction(0.2),
                new InstantAction(()-> {
                    Intake.targetPosition = robot.intake.extension.getCurrentPosition();
                    Intake.PID_ENABLED = true;
                    robot.farm.close();
                }),
                new SleepAction(0.2),
                new InstantAction(()->{
                    robot.farm.setBucketScore();
                    robot.farm.setPivot(0.5);
                    robot.intake.stopIntake();
                }),
                new SleepAction(0.6),
               new InstantAction(()-> robot.farm.setBucketScore())
        );
    }

    public void update() {
        robot.update();
    }

    public void loopTimeMeasurement(Telemetry telemetry) {
        double currTime = System.nanoTime() / 1e9;
        double delta = currTime - prevLoop;
        telemetry.addData("loop time", delta);
        telemetry.addData("hz", 1 / delta);
        telemetry.update();
        prevLoop = System.nanoTime() / 1e9;
    }
}
