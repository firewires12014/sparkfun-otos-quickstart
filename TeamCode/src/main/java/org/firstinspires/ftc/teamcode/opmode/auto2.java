package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
@Autonomous
public class auto2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoActionScheduler scheduler;
        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));
        Robot robot = new Robot(telemetry, hardwareMap);
        scheduler = new AutoActionScheduler(robot::update);
        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            scheduler.addAction(new ParallelAction(
                    //robot.lift.setLiftPosition(3000),
                    robot.intake.fourbarOut()
            ));
            scheduler.addAction(robot.drive.actionBuilder(startingPosition)
                    .strafeToLinearHeading(new Vector2d(5.5, 15), Math.toRadians(-22))
                    .build());
            scheduler.addAction(new SleepAction(3));
            scheduler.run();
            scheduler.addAction(new SleepAction(1));

            scheduler.addAction(
                    robot.intake.fourbarIn()
            );
            scheduler.addAction(new SleepAction(3));
            scheduler.run();

            return;
        }
    }
}

