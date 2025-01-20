package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.GEEKED;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.LOCKED;

import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OUTTAKE_TARGET_POSITION_IN;
import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OUTTAKE_TARGET_POSITION_OUT;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

public class Robot {
    public Intake intake;
    public Hang hang;
    public Lift lift;
    public Outtake outtake;
    public MecanumDrive drive;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        hang = new Hang(hardwareMap);
        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Intake.targetPosition = 0;
        Hang.targetPosition = 0;
        Lift.targetPosition = 0;

    }

    public void update () {
        intake.update();
        hang.update();
        lift.update();
        outtake.update();
        drive.updatePoseEstimate();

    }

    @SuppressLint("DefaultLocale")
    public Action endAuto(Telemetry telemetry, double timeout) {
        return new ParallelAction(
                new SleepAction(timeout),
                new ActionUtil.RunnableAction(()-> {
                    Pose2d pose = drive.pose;

                    double x = pose.position.x;
                    double y = pose.position.y;
                    double h = pose.heading.toDouble();

                    telemetry.addData("Robot Pose", String.format("x:%.2f \t y:%.2f \t heading:%.2f", x, y, Math.toDegrees(h)));
                    telemetry.update();

                    return true;
                })
        );
    }

    public Action depositOuttake () {
        return new SequentialAction(
                new InstantAction(outtake::flipOut),
                new SleepAction(.2),
                outtake.moveOuttakeOut()
        );
    }

    public Action dropAndReturn() {
        return new SequentialAction(
                new InstantAction(outtake::drop),
                new SleepAction(.5),
                outtake.moveOuttakeIn(),
                new InstantAction(outtake::flipIn),
                new InstantAction(outtake::hold),
                returnLift()
        );
    }

    public Action returnLift() {
        return new ActionUtil.RunnableAction(()-> {
           if (lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 8000) {
               Lift.targetPosition = lift.lift.getCurrentPosition();
               lift.lift.setPower(0);
               Lift.PID_ENABLED = true;
               return false;
           } else {
               Lift.PID_ENABLED = false;
               lift.lift.setPower(-1);
               return true;
           }
        });
    }

    public Action outtakeSpecimen () {
        return new SequentialAction(
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                lift.setTargetPositionAction(1250),
                outtake.moveOuttakeOut(),
                new InstantAction(outtake::flipSpecimen)

        );
    }

    public Action outtakeBucket () {
        return new SequentialAction(
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                lift.setTargetPositionAction(3200),
                outtake.moveOuttakeOut(),
                new InstantAction(outtake::flipOut)
        );
    }

    public Action transfer () {
        return new SequentialAction(
                new InstantAction(outtake::flipIn),
                new InstantAction(()->{
                    //lift.manualControl(-0.7);
                    lift.PID_ENABLED = false;
                    lift.lift.setPower(-0.8);
                }),
                new InstantAction(()-> intake.lock.setPosition(LOCKED)),
                new InstantAction(()->intake.spin.setPower(-0.5)),
                new InstantAction(()-> Outtake.power = -1),
                new SleepAction(.5),
                new InstantAction(()-> Outtake.power = 0),
                intake.fourbarIn(),
                intake.setTargetPositionAction(-50),
                new SleepAction(1),
                new InstantAction(()-> Outtake.power = 1),
                new InstantAction(intake::intakeOff),
                new SleepAction(1),
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                new SleepAction(1),
                new InstantAction(outtake::flipSpecimen),
                new SleepAction(.5),
                new InstantAction(()-> intake.fourbarOut()),
        new InstantAction(()->{
            //lift.manualControl(-0.7);
            lift.PID_ENABLED = true;
            lift.lift.setPower(0);
        })



        );
    }




}
