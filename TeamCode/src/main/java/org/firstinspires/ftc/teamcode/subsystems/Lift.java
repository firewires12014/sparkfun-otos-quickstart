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

import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Lift {

    public static double SPECIMEN_PICKUP = 0;
    public static double AUTO_SPECIMEN_PICKUP = 82;
    public static double SPECIMEN = 474;
    public static double LOW_BUCKET = 171;
    public static double HIGH_BUCKET = 725;
    public static double OBSERVATION_ZONE = 0;
    public static double ZERO = 0;

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

        coef = new PIDCoefficients(0.02, 0, 0);
        pid = new PIDFController(coef, 0, 0,0,(t, x, v)-> 0.0);
    }

    /**
     * Update the lift
     */
    public void update() {
        pid.setTargetPosition(targetPosition);

        //setPIDCoef(new PIDCoefficients(kP, kI, kD));

        if (PID_ENABLED) {
            newPower = this.pid.update(lift.getCurrentPosition(), lift.getVelocity());
            lift.setPower(newPower);
        }
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
