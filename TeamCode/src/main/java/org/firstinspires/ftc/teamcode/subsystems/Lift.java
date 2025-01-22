package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Lift {
    public DcMotorEx lift;
    public DcMotorEx lift2;
    public DigitalChannel limitSwitch;

    public PIDCoefficients coef;
    public PIDFController pid;

    public static double targetPosition = 0;

    public double newPower = 0.0;
    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 75;

    public static double kP = 0.0065;
    public static double kI = 0;
    public static double kD = 0;

    public static double ff = 0;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public Lift.ManualControl state = Lift.ManualControl.IDLE;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class,"lift2");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        coef = new PIDCoefficients(0.0065, 0, 0);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        pid.setTargetPosition(targetPosition);

        //setPIDCoef(new PIDCoefficients(kP, kI, kD));

        if (PID_ENABLED) {
            newPower = this.pid.update(lift.getCurrentPosition(), lift.getVelocity());
            lift.setPower(newPower);
            lift2.setPower(newPower);
        }
    }

    public void setPIDCoef(PIDCoefficients newPID) {
        this.coef.kP = newPID.kP;
        this.coef.kI = newPID.kI;
        this.coef.kD = newPID.kD;

        this.pid = new PIDFController(this.coef, 0, 0, 0, (t, x, v) -> 0.0);
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
        return new Lift.TargetPositionAction(position, false);
    }

    public Action setTargetPositionActionBlocking(int position) {
        return new Lift.TargetPositionAction(position, true);
    }

    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone)
                    state = Lift.ManualControl.ACTIVATED;

                break;
            case ACTIVATED:
                PID_ENABLED = false;

                state = Lift.ManualControl.USING;
                break;
            case USING:
                lift.setPower(joystickInput);
                lift2.setPower(joystickInput);

                if (Math.abs(joystickInput) < joystickDeadzone) state = Lift.ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = lift.getCurrentPosition();
                PID_ENABLED = true;
                state = Lift.ManualControl.IDLE;
                break;
        }
    }
}
