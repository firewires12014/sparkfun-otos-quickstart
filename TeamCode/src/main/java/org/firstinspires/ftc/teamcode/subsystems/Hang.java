package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.old.Intake;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Hang {
    public DcMotorEx hang;

    public PIDCoefficients coef;
    public PIDFController pid;

    public static double targetPosition = 0;
    public static int hangOutPosition = 3000;
    public static int hangInPosition = 0;

    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 75;

    /**
     * Constructor for the Hang class
     *
     * @param hardwareMap
     */
    public Hang(HardwareMap hardwareMap) {
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetEncoder();

        coef = new PIDCoefficients(0.0021, 0, 0);
        pid = new PIDFController(coef, 0, 0, 0, (t, x, v) -> 0.0);

    }

    /**
     * Reset the encoder
     */
    public void resetEncoder() {
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the intake to idle
     */
    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    /**
     * Update the hang target position
     */
    public void update() {
        pid.setTargetPosition(targetPosition);

        if (PID_ENABLED) {
            double newPower = this.pid.update(hang.getCurrentPosition(), hang.getVelocity());
            hang.setPower(-newPower);
        }
    }

    /**
     * Set the PID coefficients
     *
     * @param newPID
     */
    public void setPIDCoef(PIDCoefficients newPID) {
        this.coef.kP = newPID.kP;
        this.coef.kI = newPID.kI;
        this.coef.kD = newPID.kD;

        pid = new PIDFController(coef, 0, 0, 0, (t, x, v) -> 0.0);
    }

    /**
     * Check if the motor is busy
     *
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
     * Set the target position action
     *
     * @param position
     * @return
     */
    public Action setTargetPositionAction(int position) {
        return new Hang.TargetPositionAction(position, false);
    }

    /**
     * Set the target position action blocking
     *
     * @param position
     * @return
     */
    public Action setTargetPositionActionBlocking(int position) {
        return new Hang.TargetPositionAction(position, true);
    }

    /**
     * Manual control of the hang
     *
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
                hang.setPower(joystickInput);

                if (Math.abs(joystickInput) < joystickDeadzone) state = Intake.ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = hang.getCurrentPosition();
                PID_ENABLED = true;
                state = Intake.ManualControl.IDLE;
                break;
        }
    }

    /**
     * Move the hang out
     *
     * @return
     */
    public Action hangOut() { return setTargetPositionAction(hangOutPosition); }

    /**
     * Move the hang in
     *
     * @return
     */
    public Action hangIn() {
        return setTargetPositionAction(hangInPosition);
    }
}



