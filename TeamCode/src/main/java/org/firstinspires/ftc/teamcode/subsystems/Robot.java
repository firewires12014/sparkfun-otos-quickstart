package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

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
                new InstantAction(arm::clawPrime),
                new InstantAction(arm::intakePrimePosition),
                new InstantAction(intake::stopIntake),
                new SleepAction(1),
                //why wont ts (time shit) work
                new InstantAction(intake::intakeUp),
//                intake.setTargetPositionActionBlocking(0),
                intake.setTargetPositionAction(0),
//                lift.setTargetPositionActionBlocking(0),
                //block tuah
                lift.setTargetPositionAction(0),
                new SleepAction(1),
                new InstantAction(arm::grabPosition),
                new InstantAction(arm::grab)
        );
    }
//lowkey I dont know if these voids are supposed to be an action or not

    public void specIntake() {
        arm.grab();
        arm.specIntake();
        Lift.targetPosition = Lift.SPECIMEN_PICKUP;
        //i might need a wait here the claw might hit the string
        arm.drop();
    }

    public void outtakeObservation() {
        arm.grab();
        arm.observationDrop();
    }

    public void outtakeSpec() {
        arm.grab();
        Lift.targetPosition = Lift.SPECIMEN;
        arm.specDrop();
    }

    public void outtakeLowBucket() {
        arm.grab();
        Lift.targetPosition = Lift.LOW_BUCKET;
        arm.bucketPrime();
    }

    public void outtakeBucket() {
        arm.grab();
        Lift.targetPosition = Lift.HIGH_BUCKET;
        arm.bucketPrime();
    }

    public Action specDrop() {
        return new SequentialAction(
                new InstantAction(arm::drop)
                //do i use blocking idk what that is
               //finish
        );
    }

    public Action sampleDrop() {
        return new SequentialAction(
                new InstantAction(arm::bucketDrop),
                new InstantAction(arm::drop)
        );
    }
}
