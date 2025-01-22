package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Hang {
    public DcMotorEx hangmotor1;
    public Servo ptoLeft;
    public Servo ptoRight;

    PIDCoefficients coef;
    PIDFController pid;

    public static double ptoRightActive = 1;
    public static double ptoLeftActive = 1;
    public static double ptoRightInactive = 0;
    public static double ptoLeftInactive = 0;

    public static double targetPosition = 0;
    public static int hangOutPosition = 3000;
    public static int hangInPosition = 100;

    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 75;

    public Hang(HardwareMap hardwareMap) {
        hangmotor1 = hardwareMap.get(DcMotorEx.class, "lift2");
        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");

        hangmotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        hangmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangmotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetEncoder();

        coef = new PIDCoefficients(0.0018, 0, 0);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);

    }

    public void resetEncoder() {
        hangmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangmotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    public void update() {
        pid.setTargetPosition(targetPosition);

        if (PID_ENABLED) {
            double newPower = this.pid.update(hangmotor1.getCurrentPosition(), hangmotor1.getVelocity());
            hangmotor1.setPower(newPower);
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
        return new Hang.TargetPositionAction(position, false);
    }

    public Action setTargetPositionActionBlocking(int position){
        return new Hang.TargetPositionAction(position, true);
    }

    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone) state = Intake.ManualControl.ACTIVATED;

                break;
            case ACTIVATED:
                PID_ENABLED = false;

                state = Intake.ManualControl.USING;
                break;
            case USING:
                hangmotor1.setPower(joystickInput);

                if (Math.abs(joystickInput) < joystickDeadzone) state = Intake.ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = hangmotor1.getCurrentPosition();
                PID_ENABLED = true;
                state = Intake.ManualControl.IDLE;
                break;
        }
    }


    public Action engagePto() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(ptoLeft, ptoLeftActive),
                new ActionUtil.ServoPositionAction(ptoRight, ptoRightActive));
    }
    public Action hangOut () {
        return setTargetPositionActionBlocking(hangOutPosition);
    }

    public Action hangIn() {
        return setTargetPositionActionBlocking(hangInPosition);
    }

    public Action disengagePto () {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(ptoLeft, ptoLeftInactive),
                new ActionUtil.ServoPositionAction(ptoRight, ptoRightInactive));
    }
}

