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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Auto {

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

    /**
     * Constructor for the Auto class
     *
     * @param hardwareMap
     */
    public Auto(Telemetry telemetry, HardwareMap hardwareMap, Pose2d pose) {
        arm = new Arm(hardwareMap);
        hang = new Hang(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        drive = new MecanumDrive(hardwareMap, pose);
        sensors = new Sensors(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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

    /**
     * Clear the cache of the robot
     * This is used to clear the cache of the LynxModule
     */
    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Relocalize the robot with a single call to the sensors
     *
     * @return
     */
    public Action relocalize() {
        drive.pose = sensors.getSpecimenPosition(drive.pose);
        return new InstantAction(() -> drive.pose = sensors.getSpecimenPosition(drive.pose));
    }

    /**
     * Relocalize the robot with multiple call to the sensors
     *
     * @return
     */
    public Action relocalizePoll(double shutOffDistance) {
        AtomicReference<Double> x = new AtomicReference<>((double) 0);
        AtomicInteger loops = new AtomicInteger(0);
        return new SequentialAction(
                new ActionUtil.RunnableAction(() -> {
                    Pose2d p = sensors.getSpecimenPosition(drive.pose);

                    x.updateAndGet(v -> +p.position.x);
                    loops.addAndGet(1);

                    return p.position.y > shutOffDistance;
                }),
                new InstantAction(() -> {
                    Pose2d p = sensors.getSpecimenPosition(drive.pose);
                    drive.pose = new Pose2d(x.get() / loops.get(), p.position.y, drive.pose.heading.toDouble());
                })
        );
    }

    /**
     * Set the pose of the robot
     *
     * @param pose
     * @return
     */
    public Action setPose(Pose2d pose) {
        drive.pose = pose;
        return new InstantAction(() -> drive.pose = pose);
    }

    /**
     * Eject the wrong color
     *
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
     * Return the intake to the prime position
     *
     * @return
     */
    public Action returnIntake() {
        return new SequentialAction(
                new InstantAction(arm::intakePrimePosition),
                new InstantAction(arm::grab),
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
    public void outtakeSpecAuto() {
        arm.grab();
        Lift.targetPosition = Lift.SPECIMEN;
        arm.specDropAuto();
    }

    /**
     * Outtake the specimen
     */
    public void outtakeSpecArm() {
        Lift.targetPosition = Lift.SPECIMEN;
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
    public Action specDrop() {
        return new SequentialAction(
                new InstantAction(() -> {
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    arm.drop();
                })

        );
    }

    /**
     * Drop the sample into the bucket
     *
     * @return
     */
    public Action sampleDrop() {
        return new SequentialAction(
                new InstantAction(arm::bucketDrop),
                new InstantAction(arm::drop)
        );
    }

    /**
     * Score the specimen and reset to intake position
     *
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

    /**
     * Score the specimen and reset to intake position
     *
     * @return
     */
    public void outtakeSpecAutoVertical() {
        arm.wrist.setPosition(arm.WRIST_SPECIMEN_DROP);
        arm.setPivot(arm.PIVOT_SPECIMEN_DROP);
        Lift.targetPosition = Lift.SPECIMEN_DROP;
    }

    /**
     * Score the specimen and reset to intake position
     *
     * @return
     */
    public Action specScoreAuto() {
        return new SequentialAction(
                new InstantAction(() -> Lift.targetPosition = Lift.SPECIMEN_AUTO),
                new SleepAction(.5),
                new InstantAction(arm::drop)
        );
    }

    @SuppressLint("DefaultLocale")
    public Action endAuto(Telemetry telemetry, double timeout) {
        return new ParallelAction(
                new SleepAction(timeout),
                new ActionUtil.RunnableAction(() -> {
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
}
