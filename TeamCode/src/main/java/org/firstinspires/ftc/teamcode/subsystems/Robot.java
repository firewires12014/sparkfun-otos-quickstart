package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

public class Robot {

    public Arm arm;
    public Hang hang;
    public Intake intake;
    public Lift lift;
    public MecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        //hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void update() {
//        hang.update();
        intake.update();
        lift.update();
        drive.updatePoseEstimate();
    }

    public Action eject() {
        return new ActionUtil.RunnableAction(()-> {
            intake.intakeEject();
            intake.spin.setPower(Intake.INTAKE_EJECT);

            if (intake.isRightColor()) {
                intake.intakeDown();
                intake.spin.setPower(Intake.INTAKE_SPEED);
                return false;
            }

            return true;
    });
    }

    public Action transfer() {
        return new SequentialAction(
                new InstantAction(arm::prime),
                new InstantAction(arm::intakePosition),
                new InstantAction(intake::stopIntake),
                new InstantAction(intake::intakeIn),
                intake.setTargetPositionActionBlocking(0),
                lift.setTargetPositionAction(0),
                new SleepAction(.5),
                new InstantAction(arm::actualIntakePosition),
                new InstantAction(arm::grab)
        );
    }

//lowkey I dont know if these voids are supposed to be an action or not

    public void specIntake() {
        arm.specIntake();
        Lift.targetPosition = Lift.SPECIMEN_PICKUP;
    }

    public void outtakeObservation() {
        arm.observationDrop();
    }

    public void outtakeSpec() {
        Lift.targetPosition = Lift.SPECIMEN;
        arm.specDrop();
    }

    public void outtakeLowBucket() {
        Lift.targetPosition = Lift.LOW_BUCKET;
        arm.bucketPrime();
    }

    public void outtakeBucket() {
        Lift.targetPosition = Lift.HIGH_BUCKET;
        arm.bucketPrime();
    }

    public Action dropAndReturn() {
        return new SequentialAction(
                new InstantAction(arm::drop),
                new SleepAction(.0),
                new InstantAction(arm::intakePosition),
                lift.setTargetPositionAction(0) //do i use blocking idk what that is
               //finish
        );
    }

    public Action sampleDropAndReturn() {
        return new SequentialAction(
                new InstantAction(arm::bucketDrop),
                new InstantAction(arm::drop),
                new SleepAction(.0),
                new InstantAction(arm::intakePosition),
                lift.setTargetPositionAction(0) //do i use blocking idk what that is
                //finish
        );
    }
}
