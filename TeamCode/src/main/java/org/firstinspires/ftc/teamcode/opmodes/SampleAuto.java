package org.firstinspires.ftc.teamcode.opmodes;

import androidx.lifecycle.Lifecycle;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.FArm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous
//@Disabled
public class SampleAuto extends LinearOpMode {
    Robot robot;
    Vision vision;
    AutoActionScheduler scheduler;

    public static double xOffset = 1;
    public static double yOffset = -1;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    Pose2d preloadBucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample1Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample2Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d sample3Bucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));

    Pose2d sample1Pose = new Pose2d(new Vector2d(-56.33, -48.87), Math.toRadians(76.5));
    Pose2d sample2Pose = new Pose2d(new Vector2d(-58.8, -47), Math.toRadians(93));
    Pose2d sample3Pose = new Pose2d(new Vector2d(-62.15, -48.0), Math.toRadians(110));

    Pose2d cycle = new Pose2d(new Vector2d(-24.33, -10), Math.toRadians(0));
    Pose2d cycle2 = new Pose2d(new Vector2d(-24.33, -8), Math.toRadians(0));
    Pose2d cycle3 = new Pose2d(new Vector2d(-24.33, -10), Math.toRadians(0));

    Pose2d bucekt = new Pose2d(new Vector2d(-60, -56.7), Math.toRadians(45));

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        vision = new Vision(hardwareMap);
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
        scheduler = new AutoActionScheduler(this::update);

        ElapsedTime searchTimer = new ElapsedTime();

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

        Action toSub = robot.drive.actionBuilder(preloadBucket)
                .splineTo(cycle.position, cycle.heading)
                .build();

        Action toSub2 = robot.drive.actionBuilder(bucekt)
                .splineTo(cycle2.position, cycle2.heading)
                .build();

        Action toSub3 = robot.drive.actionBuilder(bucekt)
                .splineTo(cycle2.position, cycle2.heading)
                .build();

        Action toCycleBucket = robot.drive.actionBuilder(cycle)
                .setTangent(Math.toRadians(180))
                .splineTo(bucekt.position, Math.toRadians(-90) - bucekt.heading.toDouble())
                .build();

        Action toCycleBucket2 = robot.drive.actionBuilder(cycle2)
                .setTangent(Math.toRadians(180))
                .splineTo(bucekt.position, Math.toRadians(-90) - bucekt.heading.toDouble())
                .build();

        Action toCycleBucket3 = robot.drive.actionBuilder(cycle3)
                .setTangent(Math.toRadians(180))
                .splineTo(bucekt.position, Math.toRadians(-90) - bucekt.heading.toDouble())
                .build();

        // Any pre start init shi

        vision.setRed();
        robot.farm.close();
        robot.intake.intakeUp();
        // Shift into high gear and unlock PTO
        robot.setGearBoxHigh();
        robot.unlockPTO();

        while (opModeInInit() && !isStopRequested()) {
            if (gamepad2.touchpad && getRuntime() > 0.5 && vision.color.equalsIgnoreCase("blue")) {
                vision.setRed();
                resetRuntime();
            } else if (gamepad2.touchpad && getRuntime() > 0.5) {
                vision.setBlue();
                resetRuntime();
            }

            telemetry.addData("Targeting Color", vision.color);
            telemetry.update();
        }

        waitForStart();
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;
        vision.start();
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
//                    robot.pauseAuto(telemetry, ()-> gamepad1.touchpad, 1e9),
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
//                    robot.pauseAuto(telemetry, ()-> gamepad1.touchpad, 1e9),
                    ActionUtil.Delay(0.6, intake(1, 520))
            ));
            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.7, depositSecond, dropFard())
            ));

            // Cycle 3
            scheduler.addAction(new ParallelAction(
                    ActionUtil.Delay(0.2, returnLift()),
                    thirdSample,
//                    robot.pauseAuto(telemetry, ()-> gamepad1.touchpad, 1e9),
                    ActionUtil.Delay(0.5, intake(1, 520))
            ));
            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.7, depositThird, dropFard())
            ));
            scheduler.run();

            // Sub Cycle 1
            scheduler.addAction(ActionUtil.Offset(0.2, toSub, returnLift()));
            scheduler.run();

            scheduler.addAction(new InstantAction(robot.drive::flickOut));
            scheduler.addAction(new SleepAction(0.5)); // wait for it to flick before detecting
            scheduler.run();

            ArrayList<Object> dection = vision.getBlock();
            double[] offsets = (double[]) dection.get(0);

            searchTimer.reset();
            while (searchTimer.seconds() < 0.25) {
                dection = vision.getBlock();
                offsets = (double[]) dection.get(0); // TODO: Poll for a time, to allow a better estimated position
                telemetry.addData("Offset", Arrays.toString(offsets));
                telemetry.addData("Color", dection.get(1));
                telemetry.update();
                robot.intake.intakeHorizontal();
            }

            scheduler.addAction(getReadyToIntake(-offsets[0] + xOffset, offsets[1] + yOffset));
            scheduler.run();

            scheduler.addAction(intakeSub(1, 570));
            scheduler.run();

            scheduler.addAction(new ParallelAction(
                    new InstantAction(()->robot.drive.flickIn()),
                    transfer(),
                    ActionUtil.Offset(1.9, toCycleBucket, dropFard())
            ));
            scheduler.run();

            // Sub Cycle 2
            scheduler.addAction(ActionUtil.Offset(0.2, toSub2, returnLift()));
            scheduler.run();

            dection = vision.getBlock(true);
            offsets = (double[]) dection.get(0);

            searchTimer.reset();
            while (searchTimer.seconds() < 0.25) {
                dection = vision.getBlock();
                offsets = (double[]) dection.get(0); // TODO: Poll for a time, to allow a better estimated position
                telemetry.addData("Offset", Arrays.toString(offsets));
                telemetry.addData("Color", dection.get(1));
                telemetry.update();
                robot.intake.intakeHorizontal();
            }

            scheduler.addAction(getReadyToIntake(-offsets[0] + xOffset, offsets[1] + yOffset));
            scheduler.run();

            scheduler.addAction(intakeSub(1, 570));
            scheduler.run();

            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.9, toCycleBucket2, dropFard())
            ));
            scheduler.run();

            // Sub Cycle 3
            scheduler.addAction(ActionUtil.Offset(0.2, toSub3, returnLift()));
            scheduler.run();

//            scheduler.addAction(new InstantAction(robot.drive::flickOut));
//            scheduler.addAction(new SleepAction(0.5)); // wait for it to flick before detecting
//            scheduler.run();

            dection = vision.getBlock(true);
            offsets = (double[]) dection.get(0);

            searchTimer.reset();
            while (searchTimer.seconds() < 0.25) {
                dection = vision.getBlock();
                offsets = (double[]) dection.get(0); // TODO: Poll for a time, to allow a better estimated position
                telemetry.addData("Offset", Arrays.toString(offsets));
                telemetry.addData("Color", dection.get(1));
                telemetry.update();
                robot.intake.intakeHorizontal();
            }

            scheduler.addAction(getReadyToIntake(-offsets[0] + xOffset, offsets[1] + yOffset));
            scheduler.run();

            scheduler.addAction(intakeSub(1, 570));
            scheduler.run();

            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.9, toCycleBucket3, dropFard())
            ));
            scheduler.addAction(returnLift());
            scheduler.run();


            scheduler.addAction(robot.endAuto(this, telemetry, 30));
            scheduler.run();

            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }

    public Action getReadyToIntake(double x, double y) {
        // do any augmentation to x and y here
        double maxExtensionTicks = 576;
        double maxExtensionIn = 18;
        double intakeConversion = maxExtensionTicks / maxExtensionIn;

        double yOffset = 1;

        double intakeExtension = Math.max((y-yOffset) * intakeConversion, 150);

        return new ParallelAction(
                robot.drive.shiftToX(x, 0.5),
                new InstantAction(()->{
//                    robot.drive.flickIn();
                    Intake.targetPosition = intakeExtension;
                    robot.intake.intakeHorizontal();
                })
        );
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

    public Action intakeSub(double timeout, double distance) {
        return new SequentialAction(
                new ParallelAction(
                        new ActionUtil.RunnableTimedAction(timeout, ()-> {
                            robot.intake.startIntake();
                            Intake.PID_ENABLED = false;
//                    robot.intake.extension.setPower(1);
                            robot.intake.intakeDown();

                            if(robot.hasSample()) return false;

                            return !robot.hasSample();
                        }),
                        ActionUtil.Delay(0.3, new InstantAction(()-> robot.intake.extension.setPower(0.65)))
                ),
                new InstantAction(()-> {
                    Intake.targetPosition = distance;
                    Intake.PID_ENABLED = true;
                }));
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
        return new SequentialAction(
                new InstantAction(()-> {
                    robot.intake.intakeUp();
                    robot.farm.setTransfer();
                    FArm.PID_ENABLED = false;
                    robot.farm.lift.setPower(-1);
                    robot.farm.lift2.setPower(-1);
                }),
                new ActionUtil.RunnableAction(()-> {
                    if (robot.farm.lift.getCurrent(CurrentUnit.MILLIAMPS) > 7000 || robot.farm.lift2.getCurrent(CurrentUnit.MILLIAMPS) > 7000) {
                        robot.farm.resetEncoder();
                        FArm.targetPosition = 0;
                        FArm.PID_ENABLED = true;

                        return false;
                    } else return true;
                })
        );
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
