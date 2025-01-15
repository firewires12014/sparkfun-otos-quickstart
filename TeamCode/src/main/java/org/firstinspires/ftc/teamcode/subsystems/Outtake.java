package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {
    public static double FLIP_POSITION_UP = 0;
    public static double FLIP_POSITION_DOWN = 1;
    public static double GRAB_POSITION_UP = 0;
    public static double GRAB_POSITION_DOWN = 1;
    public static double OUTTAKE_TARGET_POSITION_OUT = 360;
    public static double OUTTAKE_TARGET_POSITION_IN = 0;
    public static double EXTENSION_DIRECTION = 1; // Set to positive or negative depending on direction of servo

    public CRServo extension;
    public Servo grab;
    public Servo flip;
    public AnalogInput extensionSensor;
    public double rawServoPosition; // Current servo position in degrees
    public double posCurrent; // Current servo position in degrees
    public double posPrevious; // Previous servo position in degrees
    public double delta; // Change in position in degrees

    public enum ExtensionControl {
        IN,
        OUT
    }

    /**
     * Constructor for the Outtake class
     *
     * @param hardwareMap The hardware map from the OpMode
     */
    public Outtake(HardwareMap hardwareMap) {
        extension = hardwareMap.get(CRServo.class, "outtakeExtension");
        extensionSensor = hardwareMap.get(AnalogInput.class, "extensionSensor");
        grab = hardwareMap.get(Servo.class, "grab");
        flip = hardwareMap.get(Servo.class, "flip");
    }
    
    /**
     * Update the outtake position
     */
    public void update() {
        // Calculate the current position from the sensor's voltage
        rawServoPosition = (extensionSensor.getVoltage() / 3.3) * 360;
        delta = rawServoPosition - posPrevious;
        posPrevious = rawServoPosition;
        posCurrent += delta;
    }
    
    /**
    public Outtake.moveOuttake state = Outtake.ExtensionControl.IN;
     * Move the outtake in or out based on the current state
     */
    // public void moveOuttake(ExtensionControl state) {
    //     switch (state) {
    //         case IN:
    //             moveOuttakeIn();
    //             state = Outtake.ExtensionControl.OUT;
    //             break;
    //         case OUT:
    //             moveOuttakeOut();
    //             state = Outtake.ExtensionControl.OUT;
    //             break;
    //     }
    // }

    /**
     * Get the current position of the outtake
     */
    public double getOuttakePosition() {
        return posCurrent;
    }

    /**
     * Move the outtake to a target position
     *
     * @param targetPosition The target position to move the outtake to
     */
    public Action moveOuttaketoPosition(double targetPosition) {
        return new InstantAction(() -> {
            if (getOuttakePosition() == targetPosition) {
                stopOuttake(); // Stop Servo
            } else if (getOuttakePosition() < targetPosition) {
                extension.setPower(EXTENSION_DIRECTION); // Start Servo moving in
            } else {
                extension.setPower(-EXTENSION_DIRECTION); // Start Servo moving out
            }
        });
    }

    /**
     * Move the outtake in
     */
    public Action moveOuttakeIn() {
        return new InstantAction(() -> {
            // Check if outtake is at the target position
            if (getOuttakePosition() == OUTTAKE_TARGET_POSITION_OUT) {
                stopOuttake(); // Stop Servo
            } else {
                // Flip Outtake up and start moving in
                flipOuttake("up");
                // If outtake is not at the target position start moving in
                extension.setPower(EXTENSION_DIRECTION); // Start Servo moving in
            }
        });
    }

    /**
     * Move the outtake out
     */
    public Action moveOuttakeOut() {
        return new InstantAction(() -> {
            if (getOuttakePosition() == OUTTAKE_TARGET_POSITION_IN) {
                stopOuttake(); // Stop Servo
                // Flip Outtake down when at position
                flipOuttake("down");
            } else {
                extension.setPower(-EXTENSION_DIRECTION); // Start Servo moving out
            }
        });
    }

    /**
     * Stop the outtake
     */
    public void stopOuttake() {
        extension.setPower(0); // Stop Servo
    }

    /**
     * Flip the outtake up or down
     *
     * @param direction The direction to flip the outtake
     */
    public void flipOuttake(String direction) {
        if (direction.equalsIgnoreCase("up")) {
            flip.setPosition(FLIP_POSITION_UP);
            grab.setPosition(GRAB_POSITION_UP);
        } else {
            flip.setPosition(FLIP_POSITION_DOWN);
            grab.setPosition(GRAB_POSITION_DOWN);
        }
    }

}

/*
    Call via:
    if (gamepad2.a) {
        scheduler.queueAction(robot.outtake.moveOuttakeOut());
    }
    
    if (gamepad2.b) {
        scheduler.queueAction(robot.outtake.moveOuttakeIn());
    }
*/


