package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Autonomous
@Config
@Disabled
public class RTPTesting extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;

    Pose2d start = new Pose2d(0, 0, Math.toRadians(0));

    // X is left to right (robot centric) Y is intake position NOT drive position, heading doesn't change
    public static double x = 0;
    public static double y = 5;
    public static double power = 0.3;
    Pose2d position = new Pose2d(x, y, Math.toRadians(0));

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
        // Shift into high gear and unlock PTO
        robot.setGearBoxHigh();
        robot.unlockPTO();

        waitForStart();
        resetRuntime();
        prevLoop = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
            scheduler.addAction(robot.drive.shiftToX(y, power));
            scheduler.run();

            scheduler.addAction(robot.pauseAuto(telemetry, ()-> gamepad1.touchpad, 1e9));
            scheduler.run();

            scheduler.addAction(robot.drive.shiftToX(-y, power));
            scheduler.run();

            scheduler.addAction(robot.pauseAuto(telemetry, ()-> gamepad1.touchpad, 1e9));
            scheduler.run();
        }
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
        telemetry.addLine("Telemetry Data"); // Add telemetry below this
        telemetry.addData("x error", position.position.x - robot.drive.localizer.getPose().position.x);
        telemetry.addData("y error", position.position.y - robot.drive.localizer.getPose().position.y);
        telemetry.addData("h error", Math.toDegrees(position.heading.toDouble() - robot.drive.localizer.getPose().heading.toDouble()));
        loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that

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
