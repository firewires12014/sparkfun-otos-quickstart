package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Lift {

    public static double SPECIMEN_PICKUP = 290;
    public static double ARM_FLIP_BACK = 100;
    public static double AUTO_SPECIMEN_PICKUP = 82;
    public static double SPECIMEN = 265;  // Was 1300
    public static double SPECIMEN_DROP_PRIME = 0;
    public static double SPECIMEN_DROP = 1000;
    public static double SPECIMEN_AUTO = 1125;
    public static double LOW_BUCKET = 500;
    public static double HIGH_BUCKET = 2000;
    public static double OBSERVATION_ZONE = 0;
    public static double ZERO = 0;

    public static double targetPosition = 0;
    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 50;

    public PIDCoefficients coef;
    public PIDFController pid;

    public static double kP = 0.009;
    public static double kI = 0;
    public static double kD = 0.00006;

    private double newPower = 0.0;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public ManualControl state = ManualControl.IDLE;

    public DcMotorEx lift;

    /**
     * Constructor for the Lift class
     * @param hardwareMap
     */
    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoder();

        coef = new PIDCoefficients(kP, kI, kD);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    /**
     * Update the lift
     */
    public void update() {
        if (lift.getCurrent(CurrentUnit.MILLIAMPS) > 8000) {
            targetPosition = lift.getCurrentPosition();
        }
        pid.setTargetPosition(targetPosition);
        if (PID_ENABLED) {
            newPower = this.pid.update(lift.getCurrentPosition(), lift.getVelocity());
            lift.setPower(newPower);
        }
    }

    public void updatePID() {
        coef = new PIDCoefficients(kP, kI, kD);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    public void updatePID(double kP, double kI, double kD) {
        coef = new PIDCoefficients(kP, kI, kD);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    /**
     * Reset the encoder
     */
    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    /**
     * Check if the motor is busy
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
    public void setTargetPosition(double position) {
        targetPosition = position;
    }




    /**
     * Set the target position action
     * @param position
     * @return
     */
    public Action setTargetPositionAction(int position) {
        return new TargetPositionAction(position, false);
    }

    /**
     * Set the target position action using blocking
     * @param position
     * @return
     */
    public Action setTargetPositionActionBlocking(int position) {
        return new TargetPositionAction(position, true);
    }

    /**
     * Set the power of the lift using manual control
     * @param joystickInput
     */
    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone)
                    state = ManualControl.ACTIVATED;

                break;
            case ACTIVATED:
                PID_ENABLED = false;

                state = ManualControl.USING;
                break;
            case USING:
                lift.setPower(joystickInput);

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
