package org.firstinspires.ftc.teamcode.subsystems;

import android.sax.StartElementListener;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Intake {

    public static double BLUE = 0.611;
    public static double RED = 0.279;
    public static double YELLOW = 0.345;

    public static double SAMPLE_DISTANCE = 25;

    public static String selected_color = "R";

    public static double FULL_EXTENSION = 0;
    public static double IN = 0;

    public static double MILLIAMP_SPIKE = 8000;

    public static double PIVOT_INTAKE = 0.54;
    public static double PIVOT_IN = 0.4;
    public static double PIVOT_AUTO_EJECT = 0.5;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_EJECT = 0.3;
    public static double INTAKE_REVERSE = -0.75;

    public static double FLICKER_IN = 0.0;
    public static double FLICKER_OUT = 0.4;
    public static double FLICKER_MIDDLE = 0.2;


    public static double targetPosition = 0;
    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 75;

    private PIDCoefficients coef;
    private PIDFController pid;

    private double newPower = 0.0;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public static double kP = 0.004;
    public static double kD = 0.000085;

    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    public DcMotorEx spin;
    public DcMotorEx extension;
    public Servo pivot;
    public Servo leftLight;
    public Servo rightLight;
    public Rev2mDistanceSensor subSensor;
    public RevColorSensorV3 sampleSensor;
    public Servo flicker;

    /**
     * Constructor for the Intake class
     * @param hardwareMap
     */
    public Intake(HardwareMap hardwareMap) {
        spin = hardwareMap.get(DcMotorEx.class, "spin");
//        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extension = hardwareMap.get(DcMotorEx.class, "extension");
        //extension.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoder();

        pivot = hardwareMap.get(Servo.class, "intakePivot");
        leftLight = hardwareMap.get(Servo.class, "leftLight");
        rightLight = hardwareMap.get(Servo.class, "rightLight");

        subSensor = hardwareMap.get(Rev2mDistanceSensor.class, "subDistance");
        flicker = hardwareMap.get(Servo.class, "flicker");

        sampleSensor = hardwareMap.get(RevColorSensorV3.class, "sampleColor");

        coef = new PIDCoefficients(kP, 0, kD);
        pid = new PIDFController(coef, 0, 0, 0, (t, x, v) -> 0.0);
    }

    public void updatePID() {
        coef = new PIDCoefficients(kP, 0, kD);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public void updatePID(double kP, double kI, double kD) {
        coef = new PIDCoefficients(kP, kI, kD);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public Action returnIntake() {
        return new SequentialAction(
                new InstantAction(()->{
                    PID_ENABLED = false;
                    extension.setPower(-1);
                }),
                new ActionUtil.RunnableAction(()-> extension.getCurrent(CurrentUnit.MILLIAMPS) < MILLIAMP_SPIKE || extension.getCurrentPosition() > 10),
                new InstantAction(()->{ // do this portion later in transfer
//                    targetPosition = extension.getCurrentPosition();
//                    extension.setPower(0);
//                    PID_ENABLED = true;
                })
        );
    }

    /**
     * Set the color of the leds to red
     */
    public void setColorRed() {
        selected_color = "R";
    }

    /**
     * Set the color of the leds to blue
     */
    public void setColorBlue() {
        selected_color = "B";
    }

    public void clearLED() {
        leftLight.setPosition(0);
        rightLight.setPosition(0);
    }

    /**
     * Check to see if the current color is the right color
     * @return
     */
    public boolean isRightColor() {
        if (currentColor().equalsIgnoreCase("Y")) return true;
        else if (currentColor().equalsIgnoreCase("R") || currentColor().equalsIgnoreCase("B")) {
            return selected_color.equalsIgnoreCase(currentColor());
        } else return false;
    }

    /**
     * Get the current color of the sensor
     * @return
     */
    public String currentColor() {
        int red = sampleSensor.red();
        int blue = sampleSensor.blue();
        int green = sampleSensor.green();

        if (red > blue && red > green) {
            leftLight.setPosition(RED);
            rightLight.setPosition(RED);
            return "R";
        } else if (blue > red && blue > green) {
            leftLight.setPosition(BLUE);
            rightLight.setPosition(BLUE);
            return "B";
        } else if (red > blue && green > blue) {
            leftLight.setPosition(YELLOW);
            rightLight.setPosition(YELLOW);
            return "Y";
        } else {
            // Return Orange if not sure
            leftLight.setPosition(0);
            rightLight.setPosition(0);
            return "?";
        }
    }

    /**
     * Check if the sample is in the intake
     * @return
     */
    public boolean hasSample() {
        return sampleSensor.getDistance(DistanceUnit.MM) < SAMPLE_DISTANCE;
    }

    /**
     * Start the intake
     */
    public void startIntake() {
        spin.setPower(1);
    }

    /**
     * Stop the intake
     */
    public void stopIntake() {
        spin.setPower(0);
    }

    /**
     * Reverse the intake
     */
    public void reverseIntake() {
        spin.setPower(INTAKE_REVERSE);
    }

    /**
     * Put the intake down
     */

    public void flickerIn() {
        flicker.setPosition(FLICKER_IN);
    }

    public void flickerOut(){
        flicker.setPosition(FLICKER_OUT);
    }

    public void flickerMiddle(){
        flicker.setPosition(FLICKER_MIDDLE);
    }

    public void intakeDown() {
        pivot.setPosition(PIVOT_INTAKE);
    }

    /**
     * Eject the intake
     */
    public void intakeEject() {
        pivot.setPosition(PIVOT_AUTO_EJECT);
    }

    /**
     * Put the intake up
     */
    public void intakeUp() {
        pivot.setPosition(PIVOT_IN);
    }

    /**
     * Update the intake target position
     */
    public void update() {
        pid.setTargetPosition(targetPosition);

        //setPIDCoef(new PIDCoefficients(kP, kI, kD));

        if (PID_ENABLED) {
            newPower = this.pid.update(extension.getCurrentPosition(), extension.getVelocity());
            extension.setPower(newPower);
        }
    }

    /**
     * Reset the encoders
     */
    public void resetEncoder() {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    /**
     * Check to see if the motors are busy
     * @return
     */
    public boolean isMotorBusy() {
        return Math.abs(pid.getLastError()) > tolerance;
    }

    /**
     * Action to set the target position
     */
    private class TargetPositionAction implements Action {
        int position;
        boolean blocking;

        public TargetPositionAction(int position, boolean blocking) {
            this.position = position;
            this.blocking = blocking;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (targetPosition != position) {
                targetPosition = position;
                if (blocking) {
                    return true;
                }
            }
            if (blocking) {
                return isMotorBusy();
            }
            return false;
        }
    }

    /**
     * Set the target position
     * @param position
     * @return
     */
    public Action setTargetPosition(double position) {
        return new InstantAction(() -> {
            targetPosition = position;
        });
    }

    /**
     * Set the target position of the intake
     * @param position
     * @return
     */
    public Action setTargetPositionAction(int position) {
        return new Intake.TargetPositionAction(position, false);
    }

    /**
     * Set the target position of the intake using blocking
     * @param position
     * @return
     */
    public Action setTargetPositionActionBlocking(int position) {
        return new Intake.TargetPositionAction(position, true);
    }

    /**
     * Use manual control over the intake using a FSM
     * @param joystickInput
     */
    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone)
                    state = Intake.ManualControl.ACTIVATED;

                break;
            case ACTIVATED:
                PID_ENABLED = false;

                state = Intake.ManualControl.USING;
                break;
            case USING:
                extension.setPower(joystickInput);

                if (Math.abs(joystickInput) < joystickDeadzone) state = Intake.ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = extension.getCurrentPosition();
                PID_ENABLED = true;
                state = Intake.ManualControl.IDLE;
                break;
        }
    }


}
