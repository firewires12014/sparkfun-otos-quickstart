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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Config
@Autonomous(name = "01. Specimen Auto", group = "Into the Deep")
public class SpecimenAuto extends LinearOpMode {

    // TODO: preload position fix, color ejection, timeouts for being stuck, strafing in sub (better pickup)

    public static double liftPrime = 50;
    public static double sub = 1100;
    public static double liftScore = 750;

    Robot robot;
    AutoActionScheduler scheduler;
    double startTime = System.nanoTime() / 1e9;

    Pose2d start = new Pose2d(7.1, -64, Math.toRadians(90));

    Pose2d preloadSubmersible = new Pose2d(new Vector2d(-6, -29), Math.toRadians(90));

    Pose2d splineAwayFromSubmersible = new Pose2d(new Vector2d(35, -36), Math.toRadians(90));
    Pose2d splineNextToFirstSample = new Pose2d(new Vector2d(35, -17), Math.toRadians(90));
    Pose2d splineToFirstSample = new Pose2d(new Vector2d(47, -8), Math.toRadians(-90));
    Pose2d pushFirstSample = new Pose2d(new Vector2d(50, -50), Math.toRadians(-90));

    Pose2d splineNextToSecondSample = new Pose2d(new Vector2d(46, -17.4), Math.toRadians(90));
    Pose2d splineToSecondSample = new Pose2d(new Vector2d(58, -11), Math.toRadians(-90));
    Pose2d pushSecondSample = new Pose2d(new Vector2d(54, -50), Math.toRadians(-90));

    Pose2d splineNextToThirdSample = new Pose2d(new Vector2d(54, -17.4), Math.toRadians(90));
    Pose2d splineToThirdSample = new Pose2d(new Vector2d(66.5, -18), Math.toRadians(-90));
    Pose2d pushThirdSample = new Pose2d(new Vector2d(63.5, -61), Math.toRadians(-90));


    // TODO: PICK UP FOR BEAM BRAKE CYCLES
    Pose2d grabSpecimen = new Pose2d(new Vector2d(37, -65), Math.toRadians(-90));

    // TODO: POSITION WHEN SCORING SPECIMEN
    Pose2d scoreFirstSpecimen = new Pose2d(new Vector2d(-2, -31), Math.toRadians(90));
    Pose2d scoreSecondSpecimen = new Pose2d(new Vector2d( -1, -31), Math.toRadians(90));
    Pose2d scoreThirdSpecimen = new Pose2d(new Vector2d( 1.5, -30.75), Math.toRadians(90));
    Pose2d scoreFourthSpecimen = new Pose2d(new Vector2d( 2, -30.5), Math.toRadians(90));

    //Pose2d parkPosition = new Pose2d(55, -60, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(telemetry, hardwareMap, start, true);
        scheduler = new AutoActionScheduler(this::update);

        Action toSubmersible = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(preloadSubmersible.position, preloadSubmersible.heading)
                .build();

        Action firstSample = robot.drive.actionBuilder(preloadSubmersible)
                .afterTime(2, ()-> {
                    Lift.targetPosition = 50;
                    robot.arm.wrist.setPosition(Arm.WRIST_SPECIMEN_GRAB);
                    robot.arm.setPivot(Arm.PIVOT_SPECIMEN_PICKUP);

                })
                .setReversed(true)
                .splineToConstantHeading(splineAwayFromSubmersible.position, splineAwayFromSubmersible.heading)
                .splineToConstantHeading(splineNextToFirstSample.position, splineNextToFirstSample.heading)
                .splineToConstantHeading(splineToFirstSample.position, splineToFirstSample.heading)
                .splineToConstantHeading(pushFirstSample.position, pushFirstSample.heading)

                .waitSeconds(.01)

                .splineToConstantHeading(splineNextToSecondSample.position, splineNextToSecondSample.heading)
                .splineToConstantHeading(splineToSecondSample.position, splineToSecondSample.heading)
                .splineToConstantHeading(pushSecondSample.position, pushSecondSample.heading)


                .waitSeconds(.01)

                .splineToConstantHeading(splineNextToThirdSample.position, splineNextToThirdSample.heading)
                .splineToConstantHeading(splineToThirdSample.position, splineToThirdSample.heading)
                .splineToConstantHeading(pushThirdSample.position, pushThirdSample.heading,
                        new TranslationalVelConstraint(60.0),
                        new ProfileAccelConstraint(-15, 30))
                .build();

        Action scoreFirstSpec = robot.drive.actionBuilder(new Pose2d(pushThirdSample.position, pushThirdSample.heading.toDouble() + Math.toRadians(180)))
                        .splineToConstantHeading(scoreFirstSpecimen.position, scoreFirstSpecimen.heading)
                        .build();

        Action grabSecondSpec = robot.drive.actionBuilder(new Pose2d(scoreFirstSpecimen.position, scoreFirstSpecimen.heading.toDouble() + Math.toRadians(0)))
                        .strafeToConstantHeading(grabSpecimen.position)
                                .build();

        Action scoreSecondSpec = robot.drive.actionBuilder(new Pose2d(grabSpecimen.position, grabSpecimen.heading.toDouble() + Math.toRadians(180)))
//                .strafeTo(new Vector2d(-8, -26))
                .strafeToConstantHeading(scoreSecondSpecimen.position)
                                .build();

        Action grabThirdSpec = robot.drive.actionBuilder(new Pose2d(scoreSecondSpecimen.position, scoreFirstSpecimen.heading.toDouble() + Math.toRadians(0)))

                .strafeToConstantHeading(grabSpecimen.position)
                                .build();

        Action scoreThirdSpec = robot.drive.actionBuilder(new Pose2d(grabSpecimen.position, grabSpecimen.heading.toDouble() + Math.toRadians(180)))
                        .strafeToConstantHeading(scoreThirdSpecimen.position)
                                .build();

        Action grabFourthSpec = robot.drive.actionBuilder(new Pose2d(scoreThirdSpecimen.position, scoreThirdSpecimen.heading.toDouble() + Math.toRadians(0)))
                        .strafeToConstantHeading(grabSpecimen.position)
                                .build();
        Action scoreFourthSpec = robot.drive.actionBuilder(new Pose2d(grabSpecimen.position, grabSpecimen.heading.toDouble() + Math.toRadians(180)))
                        .strafeToConstantHeading(scoreFourthSpecimen.position)
                                .build();

//        Action park = robot.drive.actionBuilder(scoreFourthSpecimen)
//                .setTangent(Math.toRadians(-90))
//                .splineTo(parkPosition.position, parkPosition.heading)
//                .build();


        robot.arm.grab();
        robot.intake.intakeUp();
        robot.arm.wrist.setPosition(Arm.WRIST_SPECIMEN_DROP);
        robot.outtakeSpecAuto();
        waitForStart();

        resetRuntime();

        while (opModeIsActive() && !isStopRequested()) {
            scheduler.addAction(new SequentialAction(
                    primeScore(),
                    ActionUtil.Offset(1.45, toSubmersible, ActionUtil.Offset(.5, score(), firstSample)),

                    // First Cycle
                    new InstantAction(robot.arm::grab),
                    ActionUtil.Offset(.2,
                            new SequentialAction(
                                    new SleepAction(0.2),
                                    robot.lift.setTargetPositionAction(500)), primeScore()),

                    ActionUtil.Offset(2.65,scoreFirstSpec, score()),

                    // Second Cycle
                    ActionUtil.Offset(.6, grabSecondSpec, robot.autoSpecGrab(1)),

                    ActionUtil.Offset(1.7, scoreSecondSpec,
                            ActionUtil.Offset(0.5, score(), ActionUtil.Offset(0.3, grabThirdSpec, robot.autoSpecGrab(1)))),

                    ActionUtil.Offset(1.7, scoreThirdSpec,
                            ActionUtil.Offset(0.5, score(), ActionUtil.Offset(0.3, grabFourthSpec, robot.autoSpecGrab(1)))),

                    ActionUtil.Offset(1.7, scoreFourthSpec, score()),

                    ActionUtil.Offset(1.5, robot.returnIntake(), new InstantAction(()-> {
                        scheduler.clearActions();
                        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    }))
            ));

            scheduler.run();
            scheduler.addAction(robot.endAuto(this, telemetry, 30));
            scheduler.run();
        }
    }

    public Action returnLift() {
        return new InstantAction(()-> {
            Lift.targetPosition = 0;
            robot.arm.intakePrimePosition();
        });
    }

    public Action primeScore() {
        return new ParallelAction(
                new InstantAction(()->Lift.targetPosition = liftPrime),
                new InstantAction(()->robot.arm.wrist.setPosition(Arm.WRIST_SPECIMEN_DROP)),
                new InstantAction(()-> robot.arm.setPivot(Arm.PIVOT_SPECIMEN_DROP)));
    }

    public Action score() {
        return new SequentialAction(
                new InstantAction(()-> {
                    Lift.targetPosition = liftScore;
                    robot.arm.wrist.setPosition(Arm.WRIST_SPECIMEN_DROP);
                }),
                new ActionUtil.RunnableAction(()-> robot.lift.lift.getCurrentPosition() < 700),
                //new SleepAction(1),
                new InstantAction(robot.arm::drop),
                new SleepAction(.3),
                new InstantAction(robot.arm::autoSpecIntake)

        );
    }

    public void update() {
        robot.update();
        //telemetry.update();
    }
}
