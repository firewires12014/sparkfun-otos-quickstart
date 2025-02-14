package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Arm {

    public static double BUCKET_TOLERANCE = 5; // in mm

    public static double OPEN = 0.75;
    public static double MIDDLE = 0.45;
    public static double CLOSED = 0.25;

    public static double WRIST_MIDDLE = 0.9;
    public static double WRIST_INTAKE = 0.38;
    public static double WRIST_SPECIMEN_GRAB = 0.25;
    public static double WRIST_SPECIMEN_DROP = 0.25;
    public static double WRIST_BUCKET_PRIME = 0.5;
    public static double WRIST_BUCKET_DROP = 0.7;

    // FORMAT: Anything prior to the decimal is the left servo and right is right servo position.
    // Example 01.99 would be left: 0.01, and right = 0.99, NOTE: only two decimal places are work
    public static double PIVOT_INTAKE = 01.99;
    public static double PIVOT_SPECIMEN_HORIZONTAL = 93.07;
    public static double PIVOT_SPECIMEN_PICKUP = 30.70;
    public static double PIVOT_SPECIMEN_PICKUP_AUTO = 27.83;
    public static double PIVOT_BUCKET = 55.45;
    public static double PIVOT_OBSERVATION = 20.80;

    public Servo leftPivot;
    public Servo rightPivot;
    public Servo wrist;
    public Servo grabber;
    public RevColorSensorV3 bucketSensor;
    public double wristPosition = 0;

    public Arm(HardwareMap hardwareMap) {
        leftPivot = hardwareMap.get(Servo.class, "leftpivot");
        rightPivot = hardwareMap.get(Servo.class, "rightpivot");
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class, "grab");
        bucketSensor = hardwareMap.get(RevColorSensorV3.class, "bucketColor");
    }

    public void setPivot(double positions) {
        String[] parts = String.format("%05.2f", positions).split("\\.");

        double left = Integer.parseInt(parts[0]) / 100.0;
        double right = Integer.parseInt(parts[1]) / 100.0;

        leftPivot.setPosition(left);
        rightPivot.setPosition(right);
    }

    public void intakePrimePosition() {
        setPivot(PIVOT_INTAKE);
        wrist.setPosition(WRIST_MIDDLE);
        wristPosition = WRIST_MIDDLE;
    }

    public void grabPosition() {
        setPivot(PIVOT_INTAKE);
        wrist.setPosition(WRIST_INTAKE);
        wristPosition = WRIST_INTAKE;
    }

    public void grab() {
        grabber.setPosition(CLOSED);
    }

    public void clawPrime() {
        grabber.setPosition(MIDDLE);
    }

    public void drop() {
        grabber.setPosition(OPEN);
    }

    public void specIntake() {
        setPivot(PIVOT_SPECIMEN_PICKUP);
        wrist.setPosition(WRIST_SPECIMEN_GRAB);
        wristPosition = WRIST_SPECIMEN_GRAB;
    }

    public boolean isBucket() {
        return bucketSensor.getDistance(DistanceUnit.MM) < BUCKET_TOLERANCE;
    }

    public void specDrop() {
        setPivot(PIVOT_SPECIMEN_HORIZONTAL);
        wrist.setPosition(WRIST_SPECIMEN_DROP);
        wristPosition = WRIST_SPECIMEN_DROP;
    }

    public void bucketPrime() {
        setPivot(PIVOT_BUCKET);
        wrist.setPosition(WRIST_BUCKET_PRIME);
    }

    public void bucketDrop() {
        setPivot(PIVOT_BUCKET);
        wrist.setPosition(WRIST_BUCKET_DROP);
    }

    public void observationDrop() {
        setPivot(PIVOT_OBSERVATION);
        wrist.setPosition(WRIST_SPECIMEN_DROP); //might be wrong wrist position
    }
}
