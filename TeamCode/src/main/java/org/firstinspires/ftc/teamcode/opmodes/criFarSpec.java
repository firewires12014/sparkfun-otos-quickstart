package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.FArm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

import java.util.function.BooleanSupplier;

@Config
@Autonomous

public class criFarSpec extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;

    Pose2d start = new Pose2d(54, -64, Math.toRadians(90));

    // Purely Positions to go to
    Pose2d scorePreload = new Pose2d(new Vector2d(58, 0), Math.toRadians(160));
    Pose2d moveAwayFromSub = new Pose2d(new Vector2d(54, -32), Math.toRadians(90));
    Pose2d grabSpec1 = new Pose2d(new Vector2d(-6, -60), Math.toRadians(90));
    Pose2d moveAwayFromWall = new Pose2d(new Vector2d(54, -32), Math.toRadians(90));
    Pose2d scoreSpec1 = new Pose2d(new Vector2d(58, -2), Math.toRadians(160));
    Pose2d grabSpec2 = new Pose2d(new Vector2d(-6, -60), Math.toRadians(90));
    Pose2d scoreSpec2 = new Pose2d(new Vector2d(58, -4), Math.toRadians(160));
    Pose2d grabSpec3 = new Pose2d(new Vector2d(-6, -60), Math.toRadians(90));
    Pose2d scoreSpec3 = new Pose2d(new Vector2d(58, -6), Math.toRadians(160));




    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
        scheduler = new AutoActionScheduler(this::update);

        // Actions to be used
        Action toSubmersible = robot.drive.actionBuilder(start)
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(scorePreload, Math.toRadians(160))
                .build();

        Action awayFromSub1 = robot.drive.actionBuilder(scorePreload)
                //.setTangent(Math.toRadians(110))
                .splineToLinearHeading(moveAwayFromSub, Math.toRadians(140))
                .build();

        Action grabFirstSpecimen = robot.drive.actionBuilder(moveAwayFromSub)
                .setTangent(90)
                .strafeToLinearHeading(grabSpec1.position, grabSpec1.heading)
                .build();

        Action awayFromWall1 = robot.drive.actionBuilder(grabSpec1)
                .strafeToLinearHeading(moveAwayFromWall.position, moveAwayFromWall.heading)
                .build();

        Action scoreFirstSpecimen = robot.drive.actionBuilder(moveAwayFromWall)
                .splineToLinearHeading(scoreSpec1, Math.toRadians(160))
                .build();

        Action awayFromSub2 = robot.drive.actionBuilder(scoreSpec1)
                .splineToLinearHeading(moveAwayFromSub, Math.toRadians(140))
                .build();

        Action grabSecondSpecimen = robot.drive.actionBuilder(moveAwayFromSub)
                .setTangent(90)
                .strafeToLinearHeading(grabSpec2.position, grabSpec2.heading)
                .build();

        Action awayFromWall2 = robot.drive.actionBuilder(grabSpec2)
                .strafeToLinearHeading(moveAwayFromWall.position, moveAwayFromWall.heading)
                .build();

        Action scoreSecondSpecimen = robot.drive.actionBuilder(moveAwayFromWall)
                .splineToLinearHeading(scoreSpec2, Math.toRadians(160))
                .build();


        Action awayFromSub3 = robot.drive.actionBuilder(scoreSpec2)
                .splineToLinearHeading(moveAwayFromSub, Math.toRadians(140))
                .build();

        Action grabThirdSpecimen = robot.drive.actionBuilder(moveAwayFromSub)
                .setTangent(90)
                .strafeToLinearHeading(grabSpec3.position, grabSpec2.heading)
                .build();

        Action awayFromWall3 = robot.drive.actionBuilder(grabSpec3)
                .strafeToLinearHeading(moveAwayFromWall.position, moveAwayFromWall.heading)
                .build();

        Action scoreThirdSpecimen = robot.drive.actionBuilder(moveAwayFromWall)
                .splineToLinearHeading(scoreSpec3, Math.toRadians(160))
                .build();





        // Any pre start init shi
        robot.farm.close();
        robot.intake.intakeUp();

        // Shift into high gear and unlock PTO
        robot.setGearBoxHigh();
        robot.unlockPTO();

        Intake.PID_ENABLED = true;
        Intake.targetPosition = 0;

        waitForStart();
        robot.farm.setAutoSpecScorePreload();
        //robot.intake.intakeHorizontal();
        robot.intake.holdIn = true;
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;

        while (opModeIsActive() && !isStopRequested()) {
            // All the actions needed should be listed below.

            scheduler.addAction(
                    new SequentialAction(
                            toSubmersible,
                            dropSpecimen()
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromSub1,
                            grabFirstSpecimen,
                            intakeSpecimen()
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromWall1,
                            ActionUtil.Offset(2, scoreFirstSpecimen, dropSpecimen())
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromSub2,
                            grabSecondSpecimen,
                            intakeSpecimen()
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromWall2,
                            ActionUtil.Offset(2, scoreSecondSpecimen, dropSpecimen())
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromSub3,
                            grabThirdSpecimen,
                            intakeSpecimen()
                    )
            );
            scheduler.run();

            scheduler.addAction(
                    new SequentialAction(
                            awayFromWall3,
                            ActionUtil.Offset(2, scoreThirdSpecimen, dropSpecimen())
                    )
            );
            scheduler.run();

            stop();

            // Example of how to pause the auto to get a value and then resume it using a button to resume
            //pause(()-> gamepad1.touchpad, telemetry);

            // Now move onto the next segment of code
            // --> insert code here <--

            // Ends the auto and keeps it running for an indefinite amount of time to move the robot to check for position
            //scheduler.addAction(robot.endAuto(this, telemetry));
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
                    robot.intake.intakeHorizontal();
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
                new ActionUtil.RunnableTimedAction(timeout, ()-> {
                    Intake.PID_ENABLED = true;
                    Intake.targetPosition = distanceIn;
                    robot.intake.intakeHorizontal();
                    robot.intake.reverseIntake();
                    //if(!robot.hasSample()) return false;
                    return true;
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
        double DELAY_BETWEEN_DROP_AND_RETURN = 0.2; // to prevent hitting the sub with the arm
        return new SequentialAction(
                new InstantAction(()-> { robot.farm.grab.setPosition(FArm.specOpen); }),
                new InstantAction(()-> { robot.farm .setSpecIntake(); })
        );
    }

    public Action intakeSpecimen() {
        return new SequentialAction(
                new InstantAction(()->robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.3, 0), 0))),
                new InstantAction(robot.farm::setSpecIntake),
                new ActionUtil.RunnableAction(()-> !robot.farm.hasSpec()), // wait until the break beam is broken
                new InstantAction(robot.farm::close),
                new SleepAction(0.2), // wait for grab to finish
                new InstantAction(()->robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0))),
                new InstantAction(robot.farm::setAutoSpecScore)// To prevent spec from slipping you can slow down the servo movement (use a linear estimator)
                //new SleepAction(.2)


        );
    }

    public void update() {
        telemetry.addData("Target Position:", FArm.targetPosition);
        telemetry.addData("Actual Position", robot.farm.lift.getCurrentPosition());
        telemetry.update();
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

