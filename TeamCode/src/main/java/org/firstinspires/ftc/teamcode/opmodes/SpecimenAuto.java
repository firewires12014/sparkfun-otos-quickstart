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

import java.util.function.BooleanSupplier;

@Config
@Autonomous
//@Disabled
public class SpecimenAuto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;

    Pose2d start = new Pose2d(-41.5, -64, Math.toRadians(90));

    // Purely Positions to go to
    Pose2d scorePreload = new Pose2d(new Vector2d(-6, -30.7), Math.toRadians(90));
    Pose2d scoreSecondSpecimen = new Pose2d(new Vector2d( -1, -30), Math.toRadians(90));
    Pose2d scoreThirdSpecimen = new Pose2d(new Vector2d( 1.5, -29.5), Math.toRadians(90));
    Pose2d scoreFourthSpecimen = new Pose2d(new Vector2d( 4, -29.5), Math.toRadians(90));
    Pose2d scoreFifthSpecimen = new Pose2d(new Vector2d(-2, -30), Math.toRadians(90));

    Pose2d parkPosition = new Pose2d(55, -60, Math.toRadians(0));

    // Actions to be used
    Action toSubmersible = robot.drive.actionBuilder(start)
            .strafeToLinearHeading(scorePreload.position, scorePreload.heading)
            .build();


    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        robot = new Robot(hardwareMap, start, LynxModule.BulkCachingMode.AUTO);
        scheduler = new AutoActionScheduler(this::update);

        // Any pre start init shi
        robot.farm.close();
        robot.intake.intakeUp();
        robot.farm.setSpecScore();
        waitForStart();
        robot.intake.intakeHorizontal();
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
            // All the actions needed should be listed below.
            scheduler.addAction(
                    new SequentialAction(
                            toSubmersible,
                            dropSpecimen())
            );
            scheduler.run(); // Do each section of code in blocks like this, not one big action, It allows for easier debugging

            // Example of how to pause the auto to get a value and then resume it using a button to resume
            pause(()-> gamepad1.touchpad, telemetry);

            // Now move onto the next segment of code
            // --> insert code here <--

            // Ends the auto and keeps it running for an indefinite amount of time to move the robot to check for position
            scheduler.addAction(robot.endAuto(this, telemetry));
            scheduler.run();

            // Not gonna do diddly squat
            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }

    public void pause(BooleanSupplier trigger, Telemetry telemetry) {
        scheduler.addAction(robot.pauseAuto(telemetry, trigger, 1e9));
        scheduler.run();
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
	