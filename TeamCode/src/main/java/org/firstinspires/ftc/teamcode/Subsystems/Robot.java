package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
import java.util.function.BooleanSupplier;

@Config
public class Robot {
    public MecanumDrive drive;
    public Intake intake;
    public FArm farm;

    public Servo lights;
    public Servo leftPTO;
    public Servo rightPTO;
    public Servo rightRack;
    public Servo leftRack;
    public Servo gearBox;

    public int colorValueRed;
    public int colorValueGreen;
    public int colorValueBlue;

    public static double YELLOW = .34;
    public static double RED = .279;
    public static double BLUE = .611;

    public static String selected_color = "RED";

    public static double leftPtoLock = 0;
    public static double leftPtoUnlock = 1;
    public static double rightPtoLock = 1;
    public static double rightPtoUnlock = 0;
    public static double leftRackUp = 0.48;
    public static double leftRackDown = 0.75;
    public static double rightRackUp = 0.52;
    public static double rightRackDown = 0.25;
    public static double gearBoxHigh = .54;
    public static double gearBoxLow = .49;

    private RevColorSensorV3 sampleColor;
//    private Rev2mDistanceSensor bucketDistance;

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
//        bucketDistance = hardwareMap.get(Rev2mDistanceSensor.class, "bucket");
        lights = hardwareMap.get(Servo.class, "lights");

        leftPTO = hardwareMap.get(Servo.class,"leftPTO");
        rightPTO = hardwareMap.get(Servo.class,"rightPTO");
        leftRack = hardwareMap.get(Servo.class, "leftRack");
        rightRack = hardwareMap.get(Servo.class, "rightRack");
        gearBox = hardwareMap.get(Servo.class, "gearBox");

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
//        return bucketDistance.getDistance(DistanceUnit.MM) < FArm.BUCKET_TOLERANCE;
        return false;
    }

    public void turnOffLight() {
        lights.setPosition(0);
    }

    public void setColorRed() {
        selected_color = "RED";
        lights.setPosition(RED);
    }

    public void setColorBlue() {
        selected_color = "BLUE";
        lights.setPosition(BLUE);
    }

    public boolean hasSample() {
        // Check for color
        if (sampleColor.getDistance(DistanceUnit.MM) < Intake.sensorDistance) {
            colorValueRed = sampleColor.red();
            colorValueBlue = sampleColor.blue();
            colorValueGreen = sampleColor.green();
            if (sampleColor.green() > 800) {
                lights.setPosition(YELLOW);
            } else if (sampleColor.red() > sampleColor.blue()) {
                lights.setPosition(RED);
            } else if (sampleColor.blue() > sampleColor.red()) {
                lights.setPosition(BLUE);
            }
        }
        return sampleColor.getDistance(DistanceUnit.MM) < Intake.sensorDistance;
    }

//    public boolean hasSample() {
//        return sampleColor.getDistance(DistanceUnit.MM) < Intake.sensorDistance;
//    }
    //I think this might break auto bc we might use the color function in auto so dont use this one ig just have it read for color twice

    public String currentColor() {
        colorValueRed = sampleColor.red();
        colorValueBlue = sampleColor.blue();
        colorValueGreen = sampleColor.green();
        if (sampleColor.green() > 800) {
            return "YELLOW";
        } else if (sampleColor.red() > sampleColor.blue()) {
            return "RED";
        } else if (sampleColor.blue() > sampleColor.red()) {
            return "BLUE";
        } else return "?";
    }

    public boolean correctColor() {
        if (currentColor().equalsIgnoreCase("YELLOW")) {
            return true;
        } else if (currentColor().equalsIgnoreCase("RED") || currentColor().equalsIgnoreCase("BLUE")) {
            return selected_color.equalsIgnoreCase(currentColor());
        } else {
            return false;
        }
    }


    // Same as the one below but doesn't have a timeout
    public Action endAuto(LinearOpMode opMode, Telemetry telemetry) {
        return endAuto(opMode, telemetry, 1e9);
    }

    @SuppressLint("DefaultLocale")
    public Action endAuto(LinearOpMode opmode, Telemetry telemetry, double timeout) {
        AtomicBoolean runtime = new AtomicBoolean(false);
        AtomicReference<Double> runTimeTime = new AtomicReference<>((double) 0);
        return new ActionUtil.RunnableTimedAction(timeout, ()-> {
            if (!runtime.get()) {
                runTimeTime.set(opmode.getRuntime());
                runtime.set(true);
            }

            // keep position updating even if previous action isn't drive related
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            double x = pose.position.x;
            double y = pose.position.y;
            double h = pose.heading.toDouble();

            telemetry.addData("Duration", runTimeTime);
            telemetry.addData("Robot Pose", String.format("x:%.2f \t y:%.2f \t heading:%.2f", x, y, Math.toDegrees(h)));
            telemetry.update();

            return true;
        });
    }

    @SuppressLint("DefaultLocale")
    public Action pauseAuto(Telemetry telemetry, BooleanSupplier trigger, double timeout) {
        return new ActionUtil.RunnableTimedAction(timeout, ()-> {
            // keep position updating even if previous action isn't drive related
            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();

            double x = pose.position.x;
            double y = pose.position.y;
            double h = pose.heading.toDouble();

//            telemetry.addData("Robot Pose", String.format("x:%.2f \t y:%.2f \t heading:%.2f", x, y, Math.toDegrees(h)));
//            telemetry.update();

            return !trigger.getAsBoolean();
        });
    }

    public void lockPTO() {
        leftPTO.setPosition(leftPtoLock);
        rightPTO.setPosition(rightPtoLock);
    }

    public void leftLockPTO() {
        leftPTO.setPosition(leftPtoLock);
    }

    public void rightLockPTO() {
        rightPTO.setPosition(rightPtoLock);
    }

    public void unlockPTO() {
        leftPTO.setPosition(leftPtoUnlock);
        rightPTO.setPosition(rightPtoUnlock);
    }

    public void leftUnlockPTO() {
        leftPTO.setPosition(leftPtoUnlock);
    }

    public void rightUnlockPTO() {
        rightPTO.setPosition(rightPtoUnlock);
    }

    public void liftRack() {
        leftRack.setPosition(leftRackUp);
        rightRack.setPosition(rightRackUp);
    }

    public void lowerRack() {
        leftRack.setPosition(leftRackDown);
        rightRack.setPosition(rightRackDown);
    }

    public void setGearBoxHigh() {
        gearBox.setPosition(gearBoxHigh);
    }

    public void setGearBoxLow() {
        gearBox.setPosition(gearBoxLow);
    }

    public boolean isLeftPtoLocked() {
        return(ActionUtil.compareDouble(leftPTO.getPosition(), leftPtoLock));
    }

    public boolean isRightPtoLocked() {
        return(ActionUtil.compareDouble(rightPTO.getPosition(), rightPtoLock));
    }




}