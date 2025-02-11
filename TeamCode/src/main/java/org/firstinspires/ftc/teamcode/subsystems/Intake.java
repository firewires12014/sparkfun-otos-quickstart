package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Intake {

    public static double SAMPLE_DISTANCE = 25;

    public static String selected_color = "R";

    public static double FULL_EXTENSION = 0;
    public static double IN = 0;

    public static double PIVOT_INTAKE = 0.54;
    public static double PIVOT_IN = 0.35;
    public static double PIVOT_AUTO_EJECT = 0.5;

    public static double INTAKE_SPEED = 1;
    public static double INTAKE_EJECT = 0.3;
    private static double INTAKE_REVERSE = -0.5;

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

    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    public DcMotorEx spin;
    private DcMotorEx extension;
    public Servo pivot;
    public Servo leftLight;
    public Servo rightLight;
    public Rev2mDistanceSensor subSensor;
    public RevColorSensorV3 sampleSensor;

    public Intake(HardwareMap hardwareMap) {
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extension = hardwareMap.get(DcMotorEx.class, "extension");
        //extension.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoder();

        pivot = hardwareMap.get(Servo.class, "intakePivot");
        leftLight = hardwareMap.get(Servo.class, "leftLight");
        rightLight = hardwareMap.get(Servo.class, "rightLight");

        subSensor = hardwareMap.get(Rev2mDistanceSensor.class, "subDistance");

        sampleSensor = hardwareMap.get(RevColorSensorV3.class, "sampleColor");

        coef = new PIDCoefficients(0.004, 0, 0);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public void setColorRed() {
        selected_color = "R";
    }

    public void setColorBlue() {
        selected_color = "B";
    }

    public boolean isRightColor() {
        if (currentColor().equalsIgnoreCase("Y")) return true;
        else if (currentColor().equalsIgnoreCase("R") || currentColor().equalsIgnoreCase("B")) {
            return selected_color.equalsIgnoreCase(currentColor());
        } else return false;
    }

    public String currentColor() {
        int red = sampleSensor.red();
        int blue = sampleSensor.blue();
        int green = sampleSensor.green();

        if (red > blue && red > green) {
            leftLight.setPosition(0.279);
            rightLight.setPosition(0.279);
            return "R";
        } else if (blue > red && blue > green) {
            leftLight.setPosition(0.611);
            rightLight.setPosition(0.611);
            return "B";
        } else if (red > blue && green > blue) {
            leftLight.setPosition(0.388);
            rightLight.setPosition(0.388);
            return "Y";
        } else {
            // Return Orange if not sure
            leftLight.setPosition(0.333);
            rightLight.setPosition(0.333);
            return "?";
        }
    }

    public boolean hasSample() {
        return sampleSensor.getDistance(DistanceUnit.MM) < SAMPLE_DISTANCE;
    }
    public void startIntake(){
        spin.setPower(1);
    }
    public void stopIntake() {
        spin.setPower(0);
    }

    public void intakeDown() {
        pivot.setPosition(PIVOT_INTAKE);
    }

    public void intakeEject() {
        pivot.setPosition(PIVOT_AUTO_EJECT);
    }

    public void intakeUp() {
        pivot.setPosition(PIVOT_IN);
    }

    public void update() {
        pid.setTargetPosition(targetPosition);

        //setPIDCoef(new PIDCoefficients(kP, kI, kD));

        if (PID_ENABLED) {
            newPower = this.pid.update(extension.getCurrentPosition(), extension.getVelocity());
            extension.setPower(newPower);
        }
    }

    public void resetEncoder() {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    public boolean isMotorBusy() {
        return Math.abs(pid.getLastError()) > tolerance;
    }

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

    public Action setTargetPositionAction(int position) {
        return new Intake.TargetPositionAction(position, false);
    }

    public Action setTargetPositionActionBlocking(int position) {
        return new Intake.TargetPositionAction(position, true);
    }

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
