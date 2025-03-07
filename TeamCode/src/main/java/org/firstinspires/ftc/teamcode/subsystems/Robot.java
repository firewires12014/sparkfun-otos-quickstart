package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Robot {

    public static double extensionTrigger = 50;
    public static double liftWait = 400;
    public static boolean isSample = true;


    public Arm arm;
    public Hang hang;
    public Intake intake;
    public Lift lift;
    public MecanumDrive drive;
    public Sensors sensors;
    List<LynxModule> allHubs;

    public enum transferState {
        IDLE,
        PRIME,
        RETRACT_ALL,
        RETRY,
        GRAB,
        CHECK_TRANSFER,
        TO_POSITION,
        DONE
    }

    public transferState TRANSFER_STATE = transferState.IDLE;

    /**
     * Constructor for the Robot class
     * @param hardwareMap
     */
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, Pose2d pose, boolean usePinpoint) {
        arm = new Arm(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        if (usePinpoint) drive = new PinpointDrive(hardwareMap, pose);
        else drive = new MecanumDrive(hardwareMap, pose);
        sensors = new Sensors(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public Robot(Telemetry telemetry, HardwareMap hardwareMap, boolean usePinpoint) {
        arm = new Arm(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        if (usePinpoint) drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        else drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        sensors = new Sensors(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public Robot(Telemetry telemetry, HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        sensors = new Sensors(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }



    public Action relocalize() {
        drive.pose = sensors.getSpecimenPosition(drive.pose);
        return new InstantAction(()->drive.pose = sensors.getSpecimenPosition(drive.pose));
    }

    public Action relocalizePoll(double shutOffDistance) {
        AtomicReference<Double> x = new AtomicReference<>((double) 0);
        AtomicInteger loops = new AtomicInteger(0);
        return new SequentialAction(
                new ActionUtil.RunnableAction(()-> {
                    Pose2d p = sensors.getSpecimenPosition(drive.pose);

                    x.updateAndGet(v -> +p.position.x);
                    loops.addAndGet(1);

                    return p.position.y > shutOffDistance;
                }),
                new InstantAction(()-> {
                    Pose2d p = sensors.getSpecimenPosition(drive.pose);
                    drive.pose = new Pose2d(x.get() / loops.get(), p.position.y, drive.pose.heading.toDouble());
                })
        );
    }

    public Action relocalizeRight() {
        drive.pose = sensors.getSpecimenRightPosition(drive.pose);
        return new InstantAction(()->drive.pose = sensors.getSpecimenPosition(drive.pose));
    }

    // The actual one this time :)
    public Action autoSpecGrab() {
        return autoSpecGrab(0.7);
    }

    public Action autoSpecGrab(double waitForDetect) {
        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(arm::grab),
                        lift.setTargetPositionAction(360),
                        new InstantAction(arm::autoSpecIntake)
                ),
                lift.setTargetPositionAction(360),
                ActionUtil.Offset(waitForDetect, new InstantAction(arm::drop), new ActionUtil.RunnableAction(()-> !sensors.hasSpec())),
                new InstantAction(arm::grab),
                new SleepAction(0.3),
                ActionUtil.Offset(0.2, lift.setTargetPositionAction(1000), new InstantAction(this::outtakeSpecTeleop)),
                lift.setTargetPositionAction(0)
        );
    }


    public Action setPose(Pose2d pose) {
        drive.pose = pose;
        return new InstantAction(()->drive.pose = pose);
    }

    /**
     * Update the robot
     * This runs on every loop of the OpMode
     */
    public void update() {
        hang.update();
        intake.update();
        lift.update();
        drive.updatePoseEstimate();
    }

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Eject the wrong color
     * @return
     */
    public Action eject() {
        return new SequentialAction(
                new InstantAction(intake::intakeUp),
                new InstantAction(intake::reverseIntake),
                new ActionUtil.RunnableAction(intake::hasSample),
                new SleepAction(0.2),
                new InstantAction(intake::intakeDown),
                new InstantAction(intake::startIntake)
        );
    }

    /**
     * Transfer from the intake into the claw
     * @return
     */
    public Action transfer() {
//        return new SequentialAction(
//                new InstantAction(arm::clawPrime),
//                new InstantAction(arm::intakePrimePosition),
//                new InstantAction(intake::stopIntake),
//                new SleepAction(.41234),
//                new ParallelAction( new InstantAction(intake::intakeUp),
//                        intake.setTargetPositionActionBlocking(0),
//                        lift.setTargetPositionActionBlocking(0)
//                ),
////                new SleepAction(.5),
//                new InstantAction(arm::grabPosition),
//                new SleepAction(.3),
//                new InstantAction(arm::grab),
//                new InstantAction(intake::reverseIntake),
//                new SleepAction(.3),
//                new InstantAction(intake::stopIntake),
//                new InstantAction(this::outtakeBucket),
//                new SleepAction(.3),
//                new InstantAction (intake :: currentColor)
//        );

        return new SequentialAction(
                new ParallelAction(
                        new InstantAction(()-> {
                            lift.updatePID(0.0065, 0, 0.00001);
                        }),
                        lift.setTargetPositionAction((int) liftWait),
                        intake.setTargetPosition(0),///////////////
                        new SequentialAction(
                                new SleepAction(0.5),
                                new InstantAction(intake::reverseIntake)
                        ),
                        new InstantAction(intake::intakeUp),
                        new InstantAction(arm::clawPrime),
                        new InstantAction(arm::intakePrimePosition),
                        new SequentialAction( //////
                                new ActionUtil.RunnableAction(()-> { // wait for intake to be to a point
                                    return intake.extension.getCurrentPosition() > extensionTrigger;
                                }),
                                lift.setTargetPositionAction(0), // maybe unblock
                                new InstantAction(intake::stopIntake)

                        )
                ),
                new InstantAction(arm::grab),
                new InstantAction(intake::reverseIntake),
                new SleepAction(0.2),
                new InstantAction(intake::stopIntake),
                new InstantAction(this::outtakeBucket),
                new ActionUtil.RunnableAction(()-> { // wait for lift to be to a point
                    return lift.lift.getCurrentPosition() < 500;
                }),
                new InstantAction(lift::updatePID)
        );
    }

    //new transfer

    public void transferFSM() {
        switch (TRANSFER_STATE) {
            case IDLE:
                if (intake.hasSample() && intake.isRightColor()) TRANSFER_STATE = transferState.PRIME;
                break;

            case PRIME:
                intake.intakeUp();
                intake.spin.setPower(0);
                Lift.targetPosition = 250;
                arm.intakePrimePosition();
                arm.clawPrime();

                Intake.PID_ENABLED = false;
                intake.extension.setPower(-1);

                if (intake.extension.getCurrent(CurrentUnit.MILLIAMPS) > 7000 || intake.extension.getCurrentPosition() < extensionTrigger) TRANSFER_STATE = transferState.RETRACT_ALL;
                break;

            case RETRACT_ALL:
                Lift.PID_ENABLED = false;
                lift.lift.setPower(-1);

                if (lift.lift.getCurrentPosition() < 10 || lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000) TRANSFER_STATE = transferState.GRAB;
                break;

            case RETRY:
                Lift.PID_ENABLED = false;
                lift.lift.setPower(-1);
                arm.clawPrime();

                if (lift.lift.getCurrentPosition() < 10 || lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000) TRANSFER_STATE = transferState.GRAB;
                break;
            case GRAB:
                long startTime = (long) (System.nanoTime() / 1e9);
                arm.grab();

                lift.lift.setPower(0);
                Lift.targetPosition = lift.lift.getCurrentPosition();
                Lift.PID_ENABLED = true;

                if (System.nanoTime() / 1e9 > startTime + 0.2) TRANSFER_STATE = transferState.CHECK_TRANSFER;
                break;

            case CHECK_TRANSFER:
                intake.reverseIntake();
                Lift.targetPosition = 450;

                if (!intake.hasSample()) TRANSFER_STATE = transferState.TO_POSITION;
                if (lift.lift.getCurrentPosition() > 400 && intake.hasSample()) TRANSFER_STATE = transferState.RETRY;
                break;

            case TO_POSITION:
                if (isSample) outtakeBucket();
                intake.stopIntake();

                intake.extension.setPower(0);
                Intake.targetPosition = intake.extension.getCurrentPosition();
                Intake.PID_ENABLED = true;



                TRANSFER_STATE = transferState.IDLE;
                break;
        }
    }



    //broken transfer
//    public void transferFSM() {
//        switch (TRANSFER_STATE) {
//            case IDLE:
//                if (intake.hasSample() && intake.isRightColor()) TRANSFER_STATE = transferState.PRIME;
//                if (!intake.hasSample()) TRANSFER_STATE = transferState.RETRACT_ALL;
//                intake.spin.setPower(1); // was 0
//                break;
//
//            case PRIME:
//                Lift.targetPosition = 250;
//                intake.intakeUp();
//                arm.intakePrimePosition();
//                arm.clawPrime();
//
//
//                Intake.PID_ENABLED = false;
//                intake.extension.setPower(-1);
//
//                if (intake.extension.getCurrent(CurrentUnit.MILLIAMPS) > 7000 || intake.extension.getCurrentPosition() < extensionTrigger) TRANSFER_STATE = transferState.RETRACT_ALL;
//                break;
//
//            case RETRACT_ALL:
//                Lift.PID_ENABLED = false;
//                lift.lift.setPower(-1);
//
//                if (!intake.hasSample()) {
//                    Intake.PID_ENABLED = false;
//                    intake.extension.setPower(-1);
//                    TRANSFER_STATE = transferState.IDLE;
//                }
//                if (lift.lift.getCurrentPosition() < 10 || lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000) TRANSFER_STATE = transferState.GRAB;
//                break;
//
//            case RETRY:
//                Lift.PID_ENABLED = false;
//                lift.lift.setPower(-1);
//                arm.clawPrime();
//
//                if (lift.lift.getCurrentPosition() < 10 || lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000) TRANSFER_STATE = transferState.GRAB;
//                break;
//
//            case GRAB:
//                long startTime = (long) (System.nanoTime() / 1e9);
//                arm.grab();
//
//                lift.lift.setPower(0);
//                Lift.targetPosition = lift.lift.getCurrentPosition();
//                Lift.PID_ENABLED = true;
//
//                if (System.nanoTime() / 1e9 > startTime + 0.2) TRANSFER_STATE = transferState.CHECK_TRANSFER;
//                break;
//
//            case CHECK_TRANSFER:
//                intake.reverseIntake();
//                Lift.targetPosition = 450;
//
//                if (!intake.hasSample()) TRANSFER_STATE = transferState.TO_POSITION;
//                if (lift.lift.getCurrentPosition() > 400 && intake.hasSample()) TRANSFER_STATE = transferState.RETRY;
//                break;
//
//            case TO_POSITION:
//                if (isSample) outtakeBucket();
//                intake.stopIntake();
//
//                intake.extension.setPower(0);
//                Intake.targetPosition = intake.extension.getCurrentPosition();
//                Intake.PID_ENABLED = true;
//
//
//
//                TRANSFER_STATE = transferState.DONE;
//                break;
//            case DONE:
//                TRANSFER_STATE = transferState.IDLE;
//        }
//    }
//
    public Action returnIntake() {
        return new SequentialAction(
                new InstantAction(arm :: intakePrimePosition),
                new InstantAction(arm :: grab),
                new SleepAction(.5),
                lift.setTargetPositionAction(0)
        );
    }

    /**
     * Outtake the observation
     */
    public void outtakeObservation() {
        arm.grab();
        arm.observationDrop();
        lift.setTargetPosition(40);
    }

    /**
     * Outtake the specimen
     */
    public void outtakeSpecTeleop() {
        arm.grab();
        Lift.targetPosition = Lift.SPECIMEN_DROP_PRIME;
        arm.specDropTeleop();
    }

    public void outtakeSpecAuto() {
        arm.grab();
        Lift.targetPosition = Lift.SPECIMEN;
        arm.specDropAuto();
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
        arm.wristPosition = arm.WRIST_BUCKET_PRIME;
        arm.grab();
        Lift.targetPosition = Lift.HIGH_BUCKET;
        arm.bucketPrime();
    }

    /**
     * Drop the specimen
     */

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
//    public Action specIntake() {
//        return new SequentialAction(
//                new InstantAction(() -> {
//                    arm.grab();
//                    Lift.targetPosition = Lift.ARM_FLIP_BACK;
//                }),
//                new SleepAction(5),
//                new InstantAction(arm::specIntake),
//                new SleepAction(5),
//                new InstantAction(() -> {
//                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
//                    arm.drop();
//                })
//        );
//    }

    /**
     * Score the specimen and reset to intake position
     * @return
     */
    public Action specScore() {
        return new SequentialAction(
                new InstantAction(() -> {
                    Lift.targetPosition = Lift.SPECIMEN_DROP;
                }),
                new ActionUtil.RunnableAction(()-> lift.lift.getCurrentPosition() < Lift.SPECIMEN_DROP - 50),
                new InstantAction(arm::drop)
        );
    }

    public void outtakeSpecAutoVertical() {
        arm.wrist.setPosition(arm.WRIST_SPECIMEN_DROP);
        arm.setPivot(arm.PIVOT_SPECIMEN_DROP);
        Lift.targetPosition = Lift.SPECIMEN_DROP;
    }

    public Action specScoreAuto() {
        return new SequentialAction(
                new InstantAction(()-> Lift.targetPosition = Lift.SPECIMEN_AUTO),
                new SleepAction(.5),
                new InstantAction(arm::drop)
        );
    }

    @SuppressLint("DefaultLocale")
    public Action endAuto(LinearOpMode opmode, Telemetry telemetry, double timeout) {
        AtomicBoolean runtime = new AtomicBoolean(false);
        AtomicReference<Double> runTimeTime = new AtomicReference<>((double) 0);
        return new ParallelAction(
                new SleepAction(timeout),
                new ActionUtil.RunnableAction(()-> {
                    if (!runtime.get()) {
                        runTimeTime.set(opmode.getRuntime());
                        runtime.set(true);
                    }

                    Pose2d pose = drive.pose;

                    double x = pose.position.x;
                    double y = pose.position.y;
                    double h = pose.heading.toDouble();

                    telemetry.addData("Duration", runTimeTime);
                    telemetry.addData("Robot Pose", String.format("x:%.2f \t y:%.2f \t heading:%.2f", x, y, Math.toDegrees(h)));
                    telemetry.update();

                    return true;
                })
        );
    }


}
