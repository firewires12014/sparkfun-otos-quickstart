package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

import java.util.Scanner;
import java.util.function.BooleanSupplier;

@Config
@Autonomous
//@Disabled
public class SpecimenAuto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;

    Pose2d start = new Pose2d(7, -64, Math.toRadians(90));

    // Purely Positions to go to
    Pose2d scorePreload = new Pose2d(new Vector2d(-6, -27), Math.toRadians(90));
    Pose2d backupFromSub = new Pose2d(new Vector2d(-6, -40), Math.toRadians(90));
    Pose2d intakeSample1 = new Pose2d(new Vector2d(28.96, -39.15), Math.toRadians(36.61));
    Pose2d spitOutSample1 = new Pose2d(new Vector2d(31.08, -44.87), Math.toRadians(-51.95));
    Pose2d intakeSample2 = new Pose2d(new Vector2d(46.5, -39.45), Math.toRadians(42.11));
    Pose2d spitOutSample2 = new Pose2d(new Vector2d(42.25, -44.37), Math.toRadians(-72.81));
    Pose2d intakeSample3 = new Pose2d(new Vector2d(50.13, -43.40), Math.toRadians(46.37));
    Pose2d spitOutSample3 = new Pose2d(new Vector2d(47.59, -45.61), Math.toRadians(-64.94));
    Pose2d turnToGrabSpec = new Pose2d(new Vector2d(47.59, -45.61), Math.toRadians(88.93));
    Pose2d grabSpec = new Pose2d(new Vector2d(36, -62.70), Math.toRadians(88.93));
    Pose2d scoreSecondSpecimen = new Pose2d(new Vector2d( -4.96, -30), Math.toRadians(90));
    Pose2d scoreThirdSpecimen = new Pose2d(new Vector2d( -.13, -30), Math.toRadians(90));
    Pose2d scoreFourthSpecimen = new Pose2d(new Vector2d( 4, -30), Math.toRadians(90));
    Pose2d scoreFifthSpecimen = new Pose2d(new Vector2d(-2, -30), Math.toRadians(90));

    Pose2d parkPosition = new Pose2d(55, -60, Math.toRadians(0));

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
        scheduler = new AutoActionScheduler(this::update);

        // Actions to be used
        Action toSubmersible = robot.drive.actionBuilder(start)
                .setTangent(-90)
                .strafeToLinearHeading(scorePreload.position, scorePreload.heading,
                            new TranslationalVelConstraint(70.0),
                            new ProfileAccelConstraint(-120, 100))
                .build();

        Action intakeFirstSample = robot.drive.actionBuilder(scorePreload)
                .strafeToLinearHeading(backupFromSub.position, backupFromSub.heading)
                .splineToLinearHeading(intakeSample1, intakeSample1.heading)
                .build();

        Action spitOutFirstSample = robot.drive.actionBuilder(intakeSample1)
                .turnTo(spitOutSample1.heading)
                .build();

        Action intakeSecondSample = robot.drive.actionBuilder(spitOutSample1)
                .turnTo(intakeSample2.heading)
                .build();

        Action spitOutSecondSample = robot.drive.actionBuilder(intakeSample2)
                .turnTo(spitOutSample2.heading)
                .build();

        Action intakeThirdSample = robot.drive.actionBuilder(spitOutSample2)
                .turnTo(intakeSample3.heading)
                .build();

        Action spitOutThirdSample = robot.drive.actionBuilder(intakeSample3)
                .turnTo(spitOutSample3.heading)
                .build();

        Action grabSecondSpecimen = robot.drive.actionBuilder(spitOutSample3)
                .turnTo(turnToGrabSpec.heading)
                .strafeToLinearHeading(grabSpec.position, grabSpec.heading)
                .build();

        Action scoreSecondSpec = robot.drive.actionBuilder(grabSpec)
                .strafeToLinearHeading(scoreSecondSpecimen.position, scoreSecondSpecimen.heading)
                .build();

        Action grabThirdSpec = robot.drive.actionBuilder(scoreSecondSpecimen)
                .strafeToLinearHeading(grabSpec.position, grabSpec.heading)
                .build();

        Action scoreThirdSpec = robot.drive.actionBuilder(grabSpec)
                .strafeToLinearHeading(scoreThirdSpecimen.position, scoreThirdSpecimen.heading)
                        .build();

        Action grabFourthSpec = robot.drive.actionBuilder(scoreThirdSpecimen)
                .strafeToLinearHeading(grabSpec.position, grabSpec.heading)
                .build();

        Action scorefourthSpec = robot.drive.actionBuilder(grabSpec)
                .strafeToLinearHeading(scoreFourthSpecimen.position, scoreFourthSpecimen.heading)
                .build();

        // Any pre start init shi
//        robot.farm.close();
        robot.intake.intakeUp();
        Intake.PID_ENABLED = true;
        Intake.targetPosition = 0;
//        robot.farm.setSpecScore();
        waitForStart();
        robot.intake.intakeHorizontal();
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;

        while (opModeIsActive() && !isStopRequested()) {
            // All the actions needed should be listed below.
            scheduler.addAction(
                    new SequentialAction(
                            toSubmersible,
                            dropSpecimen()
//                            new InstantAction(()-> { robot.farm.autoSpecIntake(); })
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            intakeFirstSample,
                            intake(1.5, 400),
                            spitOutFirstSample,
                            outtake(1.5, 400)
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            intakeSecondSample,
                            intake(1.5, 350),
                            spitOutSecondSample,
                            outtake(1.5, 250)
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            intakeThirdSample,
                            intake(1.5, 150),
                            spitOutThirdSample,
                            outtake(1.5, 150),
                            intakeEnd()
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            grabSecondSpecimen,
                            scoreSecondSpec
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            grabThirdSpec,
                            scoreThirdSpec
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            grabFourthSpec,
                            scorefourthSpec
                    )
            );
            scheduler.run();




            // Example of how to pause the auto to get a value and then resume it using a button to resume
            pause(()-> gamepad1.touchpad, telemetry);

            // Now move onto the next segment of code
            // --> insert code here <--

            // Ends the auto and keeps it running for an indefinite amount of time to move the robot to check for position
            scheduler.addAction(robot.endAuto(this, telemetry));
            //scheduler.run();

            // Not gonna do diddly squat
            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }

    public void pause(BooleanSupplier trigger, Telemetry telemetry) {
        scheduler.addAction(robot.pauseAuto(telemetry, trigger, 1e9));
        scheduler.run();
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

        public Action intakeEnd() {
            return new SequentialAction(
                    new InstantAction(()-> {
                        robot.intake.stopIntake();
                        robot.intake.intakeUp();
                        Intake.PID_ENABLED = true;
                        Intake.targetPosition = 0;
                    })
            );
        }

        public Action outtake(double timeout, double distanceIn) {
        return new SequentialAction(
                new InstantAction(()-> {
                    Intake.PID_ENABLED = true;
                    robot.intake.stopIntake();
                    Intake.targetPosition = distanceIn;
                }),
                new SleepAction(0.2),
                new ActionUtil.RunnableTimedAction(timeout, ()-> {
                    robot.intake.reverseIntake();
                    if(!robot.hasSample()) return false;
                    return robot.hasSample();
                })
        );
    }

    public Action primeScore() {
        return new InstantAction(robot.farm::setSpecScore);
    }

    public Action pickUpSample(double extensionDistance) {
        return new SequentialAction(
                new ActionUtil.RunnableTimedAction(2, ()-> {
                    robot.intake.startIntake();
                    robot.intake.intakeDown();
                    Intake.targetPosition = extensionDistance;

                    if(robot.hasSample()) return false;

                    return !robot.hasSample();
                }),
                new InstantAction(()-> {
                    robot.intake.intakeHorizontal();
                    Intake.targetPosition = 540; // Max Length
                })
        );
    }

    // Ideally not fully in, but also not far out
    public Action ejectAndRetract(double retractionDistance) {
        return new SequentialAction(
                new InstantAction(()-> {
                    robot.intake.reverseIntake();
                }),
                new SleepAction(0.2), // Delay in spitting out and returning
                new InstantAction(()-> {
                    robot.intake.stopIntake();
                    Intake.targetPosition = retractionDistance;
                })
        );
    }

    public Action dropSpecimen() {
        double DELAY_BETWEEN_DROP_AND_RETURN = 0.4; // to prevent hitting the sub with the arm
        return new SequentialAction(
                new InstantAction(robot.farm::drop),
                new SleepAction(DELAY_BETWEEN_DROP_AND_RETURN),
                new InstantAction(robot.farm::setSpecIntake)
        );
    }

    public Action intakeSpecimen() {
        return new SequentialAction(
                new InstantAction(robot.farm::setSpecIntake),
                new ActionUtil.RunnableAction(()-> !robot.farm.hasSpec()), // wait until the break beam is broken
                new InstantAction(robot.farm::close),
                new SleepAction(0.2), // wait for grab to finish
                new InstantAction(robot.farm::setSpecScore) // To prevent spec from slipping you can slow down the servo movement (use a linear estimator)
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
	