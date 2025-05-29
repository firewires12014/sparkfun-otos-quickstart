package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.function.DoubleSupplier;

@Config
public class Intake {
    public Servo pivot;
    public DcMotorEx extension;
    public DcMotorEx spin;
    DoubleSupplier axialVelo, radialVelo;

    public static double pivotMiddle = 0.53;
    public static double pivotIntake = 0.17;
    public static double pivotTransfer = 0.87;

    public static double sensorDistance = 15;
    public static double OUT_DISTANCE = 75;

    public static double targetPosition = 0;
    public static boolean PID_ENABLED = true;
    public static double tolerance = 50;

    private PIDCoefficients coef;
    private PIDFController pid;

    private final static double joystickDeadzone = 0.05;
    private double newPower = 0.0;
    public boolean holdIn = false;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public Intake.ManualControl state = Intake.ManualControl.IDLE;

    public static double kP = 0.008;
    public static double kD = 0.00035;
    public static double axialKF = 0;
    public static double radialKF = 0;

    public Intake(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "intakePivot");
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        spin = hardwareMap.get(DcMotorEx.class, "spin");

        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        coef = new PIDCoefficients(kP, 0, kD);
        pid = new PIDFController(coef, 0, 0, 0, (t, x, v) -> 0.0);

        resetEncoder();
    }

    public void updatePID() {
        pid.updatePIDCoef(kP, 0, kD);
    }

    public void updatePID(double kP, double kI, double kD) {
        pid.updatePIDCoef(kP, kI, kD);
    }

    public void update() {
        pid.setTargetPosition(targetPosition);

        if (holdIn) extension.setPower(-0.2);
        else if (PID_ENABLED) {
            newPower = this.pid.update(extension.getCurrentPosition(), extension.getVelocity());
            extension.setPower(newPower);
        }
    }

    public boolean isMotorBusy() {
        return Math.abs(pid.getLastError()) > tolerance;
    }

    public void resetEncoder() {
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    public void intakeDown() {
        pivot.setPosition(pivotIntake);
    }

    public void intakeHorizontal() {
        pivot.setPosition(pivotMiddle);
    }

    public void intakeUp() {
        pivot.setPosition(pivotTransfer);
    }

    public void startIntake() {
        spin.setPower(1);
    }

    public void stopIntake() {
        spin.setPower(0);
    }

    public void reverseIntake() {
        spin.setPower(-1);
    }

    public void setFeedforwardComponent(DoubleSupplier axialVelo, DoubleSupplier radialVelo) {
        this.axialVelo = axialVelo;
        this.radialVelo = radialVelo;
    }

    public void retractIntake() {
        targetPosition = 0;
    }

    public void retractIntakeSuperDuper() { // Big and fat | refactor
        targetPosition = -100;
    }

    public double feedforwardComponent() {
        double axial = Math.abs(axialVelo.getAsDouble()) * axialKF;
        double radial = Math.abs(radialVelo.getAsDouble()) * radialKF;

        return axial + radial;
    }

    public boolean isOuttakeOut() {
        return extension.getCurrentPosition() > OUT_DISTANCE;
    }

    public void manualControl(double joystickInput) {
        switch (state) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone)
                    state = Intake.ManualControl.ACTIVATED;
                break;
            case ACTIVATED:
                PID_ENABLED = false;
                holdIn = false;

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
