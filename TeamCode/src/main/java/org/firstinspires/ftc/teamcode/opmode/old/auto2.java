package org.firstinspires.ftc.teamcode.opmode.old;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.old.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

@Disabled
@Autonomous(name="Auto2", group="Into the Deep")
public class auto2 extends LinearOpMode {
    Robot robot;
    AutoActionScheduler scheduler;
    Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        scheduler = new AutoActionScheduler(this::update);

        while (opModeInInit() && ! isStopRequested()) {
            robot.lift.lift.setPower(-.8);
        }

        robot.lift.lift.setPower(0);



        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            scheduler.addAction(new InstantAction(()-> robot.outtake.flipSpecimen()));
            new SleepAction(3);
            scheduler.addAction(robot.drive.actionBuilder(startingPosition)
                    .strafeToLinearHeading(new Vector2d(-33.0012, -.4375), Math.toRadians(-.4502))
                    .build());
            scheduler.run();

            return;
        }
    }

    public void update(){
        robot.update();
        telemetry.addData("intakeDownSensor", robot.intake.downSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Lift Encoder Position", robot.lift.lift.getCurrentPosition());
        telemetry.update();
    }
}

