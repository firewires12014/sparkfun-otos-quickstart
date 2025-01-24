package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config

public class Intake {
    public DcMotorEx extension;
    public DcMotorEx spin;
    public Servo down;
    public Servo lock;
    public RevColorSensorV3 downSensor;
    DistanceSensor forward;

    PIDCoefficients coef;
    PIDFController pid;

    public static double targetPosition = 0;
    public static double fourbarUp = 0.74;
    public static double fourbarDown = 0.22;
    public static double fourbarResting = .6;
    public static double submerisbleBarDistance = 15;

    public static double GEEKED = 0.1;
    public static double LOCKED = 0.8;
    public static double SOMETHING_IN_BETWEEN = .76;

    public static double tolerance = 75;
    public static double joystickDeadzone = 0.05;

    public static boolean PID_ENABLED = true;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }
    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    public Intake(HardwareMap hardwareMap) {
        extension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        spin = hardwareMap.get(DcMotorEx.class, "intakeSpin");
        down = hardwareMap.get(Servo.class, "intakeDown");
        lock = hardwareMap.get(Servo.class, "intakeLock");
        downSensor = hardwareMap.get(RevColorSensorV3.class, "intakeDownSensor");
        forward = hardwareMap.get(DistanceSensor.class, "intakeForward");

        resetEncoder();

        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coef = new PIDCoefficients(0.005, 0, 0);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public void resetEncoder() {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update () {
        pid.setTargetPosition(targetPosition);

        if (PID_ENABLED) {
            double newPower = this.pid.update(extension.getCurrentPosition(), extension.getVelocity());
            extension.setPower(newPower);
        }
    }

    public void setPIDCoef(PIDCoefficients newPID) {
        this.coef.kP = newPID.kP;
        this.coef.kI = newPID.kI;
        this.coef.kD = newPID.kD;

        pid = new PIDFController(coef, 0, 0, 0, (t, x, v) -> 0.0);
    }

    public boolean isMotorBusy() {
        return Math.abs(pid.getLastError()) > tolerance;
    }

    private class TargetPositionAction implements Action {
        int position;
        boolean blocking;

        public TargetPositionAction(int position, boolean blocking){
            this.position = position;
            this.blocking = blocking;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (targetPosition != position){
                targetPosition = position;
                if(blocking){
                    return true;
                }
            }
            if (blocking){
                return isMotorBusy();
            }
            return false;
        }
    }

    public Action setTargetPositionAction(int position){
        return new Intake.TargetPositionAction(position, false);
    }

    public Action setTargetPositionActionBlocking(int position){
        return new Intake.TargetPositionAction(position, true);
    }

    public Action intakeOn () {return new ActionUtil.DcMotorExPowerAction(spin, 1);
    }

    public Action intakeOff () {
        return new ActionUtil.DcMotorExPowerAction(spin, 0);
    }

    public Action fourbarOut () {
        return new ActionUtil.ServoPositionAction(down, fourbarDown);
    }

    public Action fourbarIn () {
        return new ActionUtil.ServoPositionAction(down, fourbarUp);
    }

    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone) state = ManualControl.ACTIVATED;

                break;
            case ACTIVATED:
                PID_ENABLED = false;

                state = ManualControl.USING;
                break;
            case USING:
                extension.setPower(joystickInput);

                if (Math.abs(joystickInput) < joystickDeadzone) state = ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = extension.getCurrentPosition();
                PID_ENABLED = true;
                state = ManualControl.IDLE;
                break;
        }
    }


}
