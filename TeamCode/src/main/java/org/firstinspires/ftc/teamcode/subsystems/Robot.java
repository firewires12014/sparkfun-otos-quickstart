package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

public class Robot {

    public Arm arm;
    public Hang hang;
    public Intake intake;
    public Lift lift;
    public MecanumDrive drive;

    /**
     * Constructor for the Robot class
     * @param hardwareMap
     */
    public Robot(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        //hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    /**
     * Update the robot
     * This runs on every loop of the OpMode
     */
    public void update() {
//        hang.update();
        intake.update();
        lift.update();
        drive.updatePoseEstimate();
    }

    /**
     * Eject the wrong color
     * @return
     */
    public Action eject() {
        return new ActionUtil.RunnableAction(() -> {
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

    /**
     * Transfer from the intake into the claw
     * @return
     */
    public Action transfer() {
        return new SequentialAction(
                new InstantAction(arm::clawPrime),
                new InstantAction(arm::intakePrimePosition),
                new InstantAction(intake::stopIntake),
                new SleepAction(1),
                new InstantAction(intake::intakeUp),
                intake.setTargetPositionAction(0),
                lift.setTargetPositionAction(0),
                new SleepAction(1),
                new InstantAction(arm::grabPosition),
                new InstantAction(arm::grab)
        );
    }

    /**
     * Outtake the observation
     */
    public void outtakeObservation() {
        arm.grab();
        arm.observationDrop();
    }

    /**
     * Outtake the specimen
     */
    public void outtakeSpec() {
        arm.grab();
        Lift.targetPosition = Lift.SPECIMEN;
        arm.specDrop();
    }

    /**
     * Outtake the low bucket
     */
    public void outtakeLowBucket() {
        arm.grab();
        Lift.targetPosition = Lift.LOW_BUCKET;
        arm.bucketPrime();
    }

    /**
     * Outtake the high bucket
     */
    public void outtakeBucket() {
        arm.grab();
        Lift.targetPosition = Lift.HIGH_BUCKET;
        arm.bucketPrime();
    }

    /**
     * Drop the specimen
     */
    public Action specDrop() {
        return new SequentialAction(
                new InstantAction(arm::drop)
        );
    }

    /**
     * Drop the sample into the bucket
     * @return
     */
    public Action sampleDrop() {
        return new SequentialAction(
                new InstantAction(arm::bucketDrop),
                new InstantAction(arm::drop)
        );
    }

    /**
     * Intake the specimen from the wall and move to scoring position
     * @return
     */
    public Action specIntake() {
        return new SequentialAction(
                new InstantAction(() -> {
                    arm.grab();
                    Lift.targetPosition = Lift.ARM_FLIP_BACK;
                }),
                new SleepAction(5),
                new InstantAction(arm::specIntake),
                new SleepAction(5),
                new InstantAction(() -> {
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    arm.drop();
                })
        );
    }

    /**
     * Score the specimen and reset to intake position
     * @return
     */
    public Action specScore() {
        return new SequentialAction(
                new InstantAction(arm::drop),
                new SleepAction(.5),
                new InstantAction(() -> {
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    arm.specIntake();
                })
        );
    }

}
