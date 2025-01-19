package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;
@Autonomous
public class auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoActionScheduler scheduler;
        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));
        Robot robot = new Robot(telemetry, hardwareMap);
        scheduler = new AutoActionScheduler(robot::update);

        while (opModeInInit() && ! isStopRequested()) {
            robot.lift.lift.setPower(-.8);
            robot.intake.down.setPosition(Intake.fourbarResting);
            robot.intake.lock.setPosition(Intake.SOMETHING_IN_BETWEEN);
        }

        robot.lift.lift.setPower(0);

        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {

//            scheduler.addAction(
//                    robot.intake.fourbarOut()
//            );
            scheduler.addAction(robot.outtakeBucket());
            scheduler.addAction(robot.intake.setTargetPositionAction(900));
            scheduler.addAction(robot.intake.fourbarOut());
            scheduler.addAction(robot.drive.actionBuilder(startingPosition)
                    .strafeToLinearHeading(new Vector2d(14.63, 15.5319), Math.toRadians(-23))
                    .build());

//            scheduler.addAction(new SleepAction(30000));
//
//
//            scheduler.run();
//            scheduler.addAction(new SleepAction(1));
//
//
//            scheduler.addAction(
//                    robot.intake.fourbarIn()
//            );
//            scheduler.addAction(new SleepAction(3));

            scheduler.addAction(robot.endAuto(telemetry, 30));
            scheduler.run();
            //sleep(30000);
            return;
        }
    }
}
