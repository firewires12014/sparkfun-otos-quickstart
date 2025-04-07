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
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
@Config
//@Disabled
public class CycleTestingAuto extends LinearOpMode {
    Robot robot;
    Vision vision;
    AutoActionScheduler scheduler;

    public static double xOffset = 2;
    public static double yOffset = 0;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    Pose2d preloadBucket = new Pose2d(new Vector2d(-62.25, -58.14), Math.toRadians(54.2));
    Pose2d cycle = new Pose2d(new Vector2d(-24.33, -10), Math.toRadians(0));
    Pose2d bucekt = new Pose2d(new Vector2d(-60, -56.7), Math.toRadians(45));

    // X is left to right (robot centric) Y is intake position NOT drive position, heading doesn't change
    Pose2d cycle1Offset = new Pose2d(-10, 7, 0);

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
        vision = new Vision(hardwareMap);
        scheduler = new AutoActionScheduler(this::update);

        Action toBucket = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(preloadBucket.position, preloadBucket.heading)
                .build();

        Action toSub = robot.drive.actionBuilder(preloadBucket)
                .splineTo(cycle.position, cycle.heading)
                .build();

        Action toCycleBucket = robot.drive.actionBuilder(cycle)
                .setTangent(Math.toRadians(180))
                .splineTo(bucekt.position, Math.toRadians(-90) - bucekt.heading.toDouble())
                .build();

        // Any pre start init shi
        robot.farm.close();
        robot.intake.intakeUp();
        // Shift into high gear and unlock PTO
        robot.setGearBoxHigh();
        robot.unlockPTO();

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

            // End 4 sample
            scheduler.addAction(ActionUtil.Offset(0.2, toSub, returnLift()));
            scheduler.run();

            scheduler.addAction(new InstantAction(robot.drive::flickOut));
            scheduler.addAction(new SleepAction(1));
            scheduler.run();

            ArrayList<Object> dection = vision.getBlock();
            double[] offsets = (double[]) dection.get(0);

            resetRuntime();
            while (getRuntime() < 0.25) {
                dection = vision.getBlock();
                offsets = (double[]) dection.get(0); // TODO: Poll for a time, to allow a better estimated position
                telemetry.addData("Offset", Arrays.toString(offsets));
                telemetry.addData("Color", dection.get(1));
                telemetry.addData("runtime", getRuntime());
                telemetry.update();
                robot.intake.intakeHorizontal();
            }

            scheduler.addAction(getReadyToIntake(-offsets[0] + xOffset, offsets[1] + yOffset));
            scheduler.run();

//            scheduler.addAction(robot.pauseAuto(telemetry, ()->gamepad1.touchpad, 1e9));
//            scheduler.run();

            scheduler.addAction(intake(1, 570));
            scheduler.run();

            scheduler.addAction(new ParallelAction(
                    transfer(),
                    ActionUtil.Offset(1.9, toCycleBucket, dropFard())
            ));
            scheduler.run();

            stop();

            scheduler.addAction(robot.endAuto( this, telemetry, 30));
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
                robot.drive.shiftToX(x, 0.3),
                new InstantAction(()->{
//                    robot.drive.flickIn();
                    Intake.targetPosition = intakeExtension;
                    robot.intake.intakeHorizontal();
                })
        );
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
                new ParallelAction(
                        new ActionUtil.RunnableTimedAction(timeout, ()-> {
                            robot.intake.startIntake();
                            Intake.PID_ENABLED = false;
//                    robot.intake.extension.setPower(1);
                            robot.intake.intakeDown();
                            robot.drive.flickIn();

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

    public Action returnLift() {
        return new InstantAction(()->{
                robot.farm.setTransfer();
                robot.intake.intakeUp();
        });
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
