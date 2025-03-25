package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class Robot {
    public MecanumDrive drive;
    public Intake intake;
    public FArm farm;

    public Servo leftLight;
    public Servo rightLight;
    public Servo leftPTO;
    public Servo rightPTO;
    public Servo rightRack;
    public Servo leftRack;

    public int colorValueRed;
    public int colorValueGreen;
    public int colorValueBlue;

    public static double yellow = .34;
    public static double PTOLockLeft = 0;
    public static double PTOLockRight = 0;
    public static double PTOUnlockLeft = 1;
    public static double PTOUnlockRight = 1;
    public static double rightRackLift = .2;
    public static double leftRackLift = .2;
    public static double rightRackLower = .2;
    public static double leftRackLower = .2;
    private RevColorSensorV3 sampleColor;
    private Rev2mDistanceSensor bucketDistance;

//    for threading
//    private final Object sensorLock = new Object();
//    @GuardedBy("sensorLock")

    List<LynxModule> allHubs;
    LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.AUTO;

    // Default Constructor
    public Robot(HardwareMap hardwareMap) {
        new Robot(hardwareMap, new Pose2d(0,0,0), LynxModule.BulkCachingMode.AUTO);
    }

    public Robot(HardwareMap hardwareMap, Pose2d startPose, LynxModule.BulkCachingMode lynxReadMode) {
        intake = new Intake(hardwareMap);
        farm = new FArm(hardwareMap);
        drive = new MecanumDrive(hardwareMap, startPose);

        sampleColor = hardwareMap.get(RevColorSensorV3.class, "sampleColor");
        bucketDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bucket");
        leftLight = hardwareMap.get(Servo.class, "leftLight");
        rightLight = hardwareMap.get(Servo.class, "rightLight");

//        leftPTO = hardwareMap.get(Servo.class, "leftPTO");
//        rightPTO = hardwareMap.get(Servo.class, "rightPTO");
//        leftRack = hardwareMap.get(Servo.class, "leftRack");
//        rightRack = hardwareMap.get(Servo.class, "rightRack");


        // absolute tom foolery, slide accel compensation do NOT use
//        intake.setFeedforwardComponent(()-> { // TODO: COULD SLOW DOWN LOOPS
//            PoseVelocity2d velo = drive.localizer.update();
//            double x = Math.pow(velo.component1().x, 2);
//            double y = Math.pow(velo.component1().y, 2);
//            return Math.sqrt(x + y);
//        }, ()-> drive.localizer.update().angVel);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(lynxReadMode);
        }

        bulkCachingMode = lynxReadMode;
    }

    public void update() {
        //drive.updatePoseEstimate(); // TODO: COULD SLOW DOWN LOOPS
        intake.update();
        farm.update();
    }

    public void clearBulkCache() {
        if (bulkCachingMode.equals(LynxModule.BulkCachingMode.MANUAL)) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        } // else do nothing
    }

    public boolean inRangeOfBucket() {
        return bucketDistance.getDistance(DistanceUnit.MM) < FArm.BUCKET_TOLERANCE;
    }

    public void turnOffLight() {
        leftLight.setPosition(0);
        rightLight.setPosition(0);
    }

    public boolean hasSample() {
        // Check for color
        if (sampleColor.getDistance(DistanceUnit.MM) < Intake.sensorDistance) {
            colorValueRed = sampleColor.red();
            colorValueBlue = sampleColor.blue();
            colorValueGreen = sampleColor.green();
            if (sampleColor.green() > 800) {
                leftLight.setPosition(yellow);
                rightLight.setPosition(yellow);
            } else if (sampleColor.red() > sampleColor.blue()) {
                leftLight.setPosition(.279);
                rightLight.setPosition(.279);
            } else if (sampleColor.blue() > sampleColor.red()) {
                leftLight.setPosition(.611);
                rightLight.setPosition(.611);
            }        }

        return sampleColor.getDistance(DistanceUnit.MM) < Intake.sensorDistance;
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

                    Pose2d pose = drive.localizer.getPose();

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

    public void lockPTO() {
        leftPTO.setPosition(PTOLockLeft);
        rightPTO.setPosition(PTOLockRight);
    }

    public void unlockPTO() {
        leftPTO.setPosition(PTOUnlockLeft);
        rightPTO.setPosition(PTOUnlockRight);
    }

    public void liftRack() {
        leftRack.setPosition(leftRackLift);
        rightRack.setPosition(rightRackLift);
    }

    public void lowerRack() {
        leftRack.setPosition(leftRackLower);
        rightRack.setPosition(rightRackLower);
    }

}
