package org.firstinspires.ftc.teamcode.subsystems.old;

import static org.firstinspires.ftc.teamcode.subsystems.old.Intake.GEEKED;
import static org.firstinspires.ftc.teamcode.subsystems.old.Intake.LOCKED;

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
import org.firstinspires.ftc.teamcode.util.ActionUtil;

public class Robot {
    public Intake intake;
//    public Hang hang;
    public Lift lift;
    public Outtake outtake;
    public MecanumDrive drive;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
//      hang = new Hang(hardwareMap);
        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Intake.targetPosition = 0;
//      Hang.targetPosition = 0;
        Lift.targetPosition = 0;

    }

    public void update () {
        intake.update();
//      hang.update();
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

    public Action awaitGamepad(boolean trigger) {
        return new ActionUtil.RunnableAction(()-> trigger);
    }

    public Action depositOuttake () {
        return new SequentialAction(
                new InstantAction(outtake::flipOut),
                new SleepAction(.2),
                outtake.moveOuttakeOut()
        );
    }

    public Action dropAndReturnTeleop() {
        return new SequentialAction(
                new InstantAction(outtake::drop),
                new SleepAction(.5),
                outtake.moveOuttakeIn(),
                new InstantAction(outtake::flipIn),
                new InstantAction(outtake::hold),
                returnLiftTeleop()
        );
    }

    public Action returnLiftTeleop() {
        return new ActionUtil.RunnableAction(()-> {
           if (lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 5000) {
               Lift.targetPosition = lift.lift.getCurrentPosition();
               lift.lift.setPower(0);
               lift.lift2.setPower(0);
               Lift.PID_ENABLED = true;

               outtake.hold();

               lift.resetEncoder();
               return false;
           } else {
               Lift.PID_ENABLED = false;
               lift.lift.setPower(-1);
               lift.lift2.setPower(-1);
               return true;
           }
        });
    }

    public Action dropAndReturnAuto() {
        return new SequentialAction(
                new InstantAction(outtake::drop),
                new SleepAction(.2),
                outtake.moveOuttakeIn(),
                new InstantAction(outtake::flipIn),
//                new InstantAction(outtake::hold),
                new InstantAction(intake::fourbarOut),
                returnLiftAuto()
        );
    }

    public Action returnLiftAuto() {
//        return new ActionUtil.RunnableAction(()-> {
//            if (lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000 || lift.lift.getCurrentPosition() < 100) {
//                Lift.targetPosition = lift.lift.getCurrentPosition();
//                lift.lift.setPower(0);
//                lift.lift2.setPower(0);
//                Lift.PID_ENABLED = true;
//                lift.resetEncoder();
//                return false;
//            } else {
//                Lift.PID_ENABLED = false;
//                lift.lift.setPower(-1);
//                lift.lift2.setPower(-1);
//                return true;
//            }
//        });
        return new ActionUtil.RunnableAction(()-> {
            update();

            if (lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 5000 && lift.lift.getCurrentPosition() < 600) {
                Lift.targetPosition = lift.lift.getCurrentPosition();
                lift.lift.setPower(0);
                lift.lift2.setPower(0);
                Lift.PID_ENABLED = true;
                return false;
            } else {
                Lift.PID_ENABLED = false;
                lift.lift.setPower(-1);
                lift.lift2.setPower(-1);
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

    public Action outtakeLowBucket () {
        return new SequentialAction(
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                lift.setTargetPositionAction(1812),
                outtake.moveOuttakeOut(),
                new InstantAction(outtake::flipSpecimen)
        );
    }

    public Action autoPark () {
        return new SequentialAction(
                new InstantAction(()-> Outtake.power = -1),
                lift.setTargetPositionAction(1280));

    }

    public Action outtakeBucket () {
        return new SequentialAction(
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                lift.setTargetPositionAction(3200),
                outtake.moveOuttakeOut(),
                //new SleepAction(.2), // since outtake waits a second this can be commented out
                new InstantAction(outtake::flipOut)
        );
    }

    public Action outtakeBucketAuto (double duration, int position) {
        return new SequentialAction(
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                lift.setTargetPositionAction(position),
                new SleepAction(duration),
                outtake.moveOuttakeOut(),
                new InstantAction(outtake::flipOut)
        );
    }

    public Action quickSpit() {
        return new SequentialAction(
                new InstantAction(()-> intake.spin.setPower(1)),
                new SleepAction(1),
                intake.intakeOff()
        );
    }

    public Action droppa() {
        return new SequentialAction(
                new InstantAction(outtake::drop)
        );
    }

    public Action transfer () {
        return new SequentialAction(
                new ParallelAction(intake.fourbarIn(), new InstantAction(intake::locked), new InstantAction(outtake::flipIn), new InstantAction(outtake::drop), intake.intakeOn()),
                new InstantAction(()-> {
                    Outtake.power = 1;
                }),
                new SleepAction(0.3), // wait for fourbar to come in
                intake.setTargetPositionActionBlocking(-15),
                new InstantAction(outtake::hold),
                new SleepAction(0.25),
                new InstantAction(intake::geeked),
                new InstantAction(outtake::flipPrimed),
                intake.intakeOff(),
                new InstantAction(()-> {
                    Outtake.power = 0;
                })

        );
    }

    public Action getIntakeReady(double extensionDistace) {
        return new SequentialAction(
                intake.fourbarOut(),
                new InstantAction(intake::somethingInBetween),
                intake.autoExtend(0.5, extensionDistace),
                intake.setTargetPositionAction(1200),
                new SleepAction(0.7),
                intake.fourbarIn()
        );
    }

    public Action transferOld () {
        return new SequentialAction(
                new InstantAction(outtake::flipIn),
                new InstantAction(()->{
                    //lift.manualControl(-0.7);
                    lift.PID_ENABLED = false;
                    lift.lift.setTargetPosition(80);
                    lift.lift2.setTargetPosition(80);
                }),
                new InstantAction(()-> intake.lock.setPosition(LOCKED)),
                new InstantAction(()->intake.spin.setPower(-0.5)),
                new InstantAction(()-> Outtake.power = -1),
                new SleepAction(.25),
                new InstantAction(()-> Outtake.power = 0),
                intake.fourbarIn(),
                intake.setTargetPositionAction(-50),
                new SleepAction(.5),
                new InstantAction(()-> Outtake.power = 1),
                new InstantAction(intake::intakeOff),
                new SleepAction(.5),
                new InstantAction(()->intake.lock.setPosition(GEEKED)),
                new SleepAction(.5),
                new InstantAction(outtake::flipSpecimen),
                new SleepAction(.5),
//                new InstantAction(()-> lift.lift.setTargetPosition(500)),
//                new InstantAction(()-> lift.lift2.setTargetPosition(500)),
                new InstantAction(()-> intake.fourbarOut()),
                new InstantAction(()->{
                    //lift.manualControl(-0.7);
                    lift.PID_ENABLED = true;
                    lift.lift.setPower(0);
                    lift.lift2.setPower(0);
                })



        );
    }


}
