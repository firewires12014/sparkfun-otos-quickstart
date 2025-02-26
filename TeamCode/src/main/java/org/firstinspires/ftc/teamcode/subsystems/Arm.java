package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Arm {

    public static double BUCKET_TOLERANCE = 5; // in mm

    public static double OPEN = 0.90;
    public static double MIDDLE = 0.77;
    public static double CLOSED = .67;

    public static double WRIST_MIDDLE = 0.32;
    public static double WRIST_INTAKE = 0.38;
    public static double WRIST_SPECIMEN_GRAB = 0.25;
    public static double WRIST_SPECIMEN_DROP = 0.2;
    public static double WRIST_SPECIMEN_DROP_AUTO = .25;
    public static double WRIST_BUCKET_PRIME = 0.45;
    public static double WRIST_BUCKET_DROP = 0.73;
    public static double WRIST_AUTO_PICKUP = 0.1;

    // FORMAT: Anything prior to the decimal is the left servo and right is right servo position.
    // Example 01.99 would be left: 0.01, and right = 0.99, NOTE: only two decimal places are work
    public static double PIVOT_INTAKE = 01.99;
    public static double PIVOT_SPECIMEN_DROP = 70.30;
    public static double PIVOT_SPECIMEN_DROP_AUTO = 93.07;
    public static double PIVOT_SPECIMEN_PICKUP = 30.70;
    public static double PIVOT_SPECIMEN_PICKUP_AUTO = 18.82;
    public static double PIVOT_BUCKET = 55.45;
    public static double PIVOT_OBSERVATION = 34.66;
    public static double PIVOT_AUTO_CLIP = 75.25;

   // public static double PIVOT_AUTO_UP =

    public Servo leftPivot;
    public Servo rightPivot;
    public Servo wrist;
    public Servo grabber;
    public Rev2mDistanceSensor bucketSensor;
    public static double wristPosition = 0;


    /**
     * Constructor for the Arm class
     * @param hardwareMap
     */
    public Arm(HardwareMap hardwareMap) {
        leftPivot = hardwareMap.get(Servo.class, "leftpivot");
        rightPivot = hardwareMap.get(Servo.class, "rightpivot");
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class, "grab");
        bucketSensor = hardwareMap.get(Rev2mDistanceSensor.class, "bucket");
    }

    /**
     * Set the position of the pivot
     *      FORMAT: Anything prior to the decimal is the left servo and right is right servo position.
     *      Example 01.99 would be left: 0.01, and right = 0.99, NOTE: only two decimal places are work
     * @param positions
     */
    public void setPivot(double positions) {
        String[] parts = String.format("%05.2f", positions).split("\\.");

        double left = Integer.parseInt(parts[0]) / 100.0;
        double right = Integer.parseInt(parts[1]) / 100.0;

        leftPivot.setPosition(left);
        rightPivot.setPosition(right);
    }

    /**
     * Set the position of the pivot to the intake prime position
     */
    public void intakePrimePosition() {
        setPivot(PIVOT_INTAKE);
        wrist.setPosition(WRIST_MIDDLE);
        wristPosition = WRIST_MIDDLE;
    }

    /**
     * Set the position of the pivot to the grab position
     */
    public void grabPosition() {
        setPivot(PIVOT_INTAKE);
        wrist.setPosition(WRIST_INTAKE);
        wristPosition = WRIST_INTAKE;
    }

    public double getBucketDistance() {
        return bucketSensor.getDistance(DistanceUnit.MM);
    }

    /**
     * Close the grabber/claw
     */
    public void grab() {
        grabber.setPosition(CLOSED);
    }

    /**
     * Open the grabber/claw to the middle position
     */
    public void clawPrime() {
        grabber.setPosition(MIDDLE);
    }

    /**
     * Open the grabber/claw to the open position
     */
    public void drop() {
        grabber.setPosition(OPEN);
    }

    /**
     * Set the position of the pivot to the specimen pickup position
     */
    public void specIntake() {
        setPivot(PIVOT_SPECIMEN_PICKUP);
        wrist.setPosition(WRIST_SPECIMEN_GRAB);
        wristPosition = WRIST_SPECIMEN_GRAB;

//        autoSpecIntake();
    }

    public void autoSpecIntake() {
        setPivot(PIVOT_SPECIMEN_PICKUP_AUTO);
        wrist.setPosition(WRIST_AUTO_PICKUP);
        wristPosition = WRIST_AUTO_PICKUP;
    }


    /**
     * Check the distance of the bucket sensor to the bucket
     * @return
     */
    public boolean isBucket() {
        return bucketSensor.getDistance(DistanceUnit.MM) < BUCKET_TOLERANCE;
    }

    /**
     * Set the position of the pivot to the specimen drop position
     */
    public void specDropTeleop() {
        setPivot(PIVOT_SPECIMEN_DROP);
        wrist.setPosition(WRIST_SPECIMEN_DROP);
        wristPosition = WRIST_SPECIMEN_DROP;
    }

    public void specDropAuto() {
        setPivot(PIVOT_SPECIMEN_DROP_AUTO);
        wrist.setPosition(WRIST_SPECIMEN_DROP_AUTO);
        wristPosition = WRIST_SPECIMEN_DROP_AUTO;
    }



    /**
     * Set the position of the pivot to the bucket prime position
     */
    public void bucketPrime() {
        setPivot(PIVOT_BUCKET);
        wrist.setPosition(WRIST_BUCKET_PRIME);
        wristPosition = WRIST_BUCKET_DROP;
    }

    /**
     * Set the position of the pivot to the bucket drop position
     */
    public void bucketDrop() {
        setPivot(PIVOT_BUCKET);
        wrist.setPosition(WRIST_BUCKET_DROP);
        wristPosition = WRIST_BUCKET_DROP;
    }

    /**
     * Set the position of the pivot to the observation drop position
     */
    public void observationDrop() {
        setPivot(PIVOT_OBSERVATION);
        wrist.setPosition(WRIST_SPECIMEN_DROP); //might be wrong wrist position
        wristPosition = WRIST_SPECIMEN_DROP;
    }
}
