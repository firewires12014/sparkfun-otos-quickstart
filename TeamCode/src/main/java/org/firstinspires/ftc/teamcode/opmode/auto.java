package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.GEEKED;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
@Autonomous
public class auto extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;
    Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        scheduler = new AutoActionScheduler(this::update);

        while (opModeInInit() && ! isStopRequested()) {
            robot.lift.lift.setPower(-.8);
            robot.intake.down.setPosition(Intake.fourbarResting);
            robot.intake.lock.setPosition(Intake.SOMETHING_IN_BETWEEN);
        }

        robot.lift.lift.setPower(0);

        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            preload();
            returnLiftAndIntake();

            scheduler.addAction(robot.endAuto(telemetry, 30));
            scheduler.run();
            return;
        }
    }

    public void preload() {
        scheduler.addAction(robot.outtakeBucket());
        scheduler.addAction(robot.intake.setTargetPositionAction(600));
        scheduler.addAction(robot.intake.fourbarOut());
        scheduler.addAction(robot.drive.actionBuilder(startingPosition)
                .strafeToLinearHeading(new Vector2d(17.7, 21.86), Math.toRadians(-25.4))
                .build());
        scheduler.addAction(new SleepAction(1));
        scheduler.addAction(new InstantAction(()-> robot.outtake.grab.setPosition(Outtake.GRAB_POSITION_DOWN)));
        scheduler.addAction(new InstantAction(()-> {
            robot.intake.spin.setPower(-1);
            robot.intake.fourbarOut();
        }));
        scheduler.addAction(new InstantAction(()-> {
            Intake.PID_ENABLED = false;
            robot.intake.extension.setPower(0.5);
            robot.intake.lock.setPosition(Intake.SOMETHING_IN_BETWEEN);
        }));
        scheduler.addAction(robot.dropAndReturn());
        scheduler.run();
    }

    public void returnLiftAndIntake() {
        scheduler.addAction(new InstantAction(()-> {
            Lift.PID_ENABLED = false;
            robot.lift.lift.setPower(-1);
            robot.intake.intakeOff();
            robot.intake.fourbarIn();

            robot.intake.extension.setPower(0);
            Intake.PID_ENABLED = true;

            Outtake.power = 1; // could cause issues if it isn't set to zero later
        }));
        while (!(robot.intake.downSensor.getDistance(DistanceUnit.MM) < 20)) {}
        scheduler.run();

        scheduler.addAction(robot.transfer());
        scheduler.addAction(new InstantAction(()-> {
            Lift.PID_ENABLED  = true;
            robot.intake.spin.setPower(0);
        }));

        while (!(robot.lift.lift.getCurrentPosition() < 0)) {} // maybe set higher or lower to make it transfer faster
        scheduler.run();
    }

    public void scoreSecond() {
        scheduler.addAction(new InstantAction(()->robot.intake.lock.setPosition(GEEKED)));
        scheduler.addAction(robot.outtakeBucket());
    }

    public void update(){
        robot.update();

        telemetry.addData("Lift Encoder Position", robot.lift.lift.getCurrentPosition());
        telemetry.update();
    }
}
