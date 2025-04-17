package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class FArm {
    public static double BUCKET_TOLERANCE = 200;

    public static double targetPosition = 0;
    public static boolean PID_ENABLED = true;
    public static double joystickDeadzone = 0.05;
    public static double tolerance = 75; //

    public static double PIVOT_OFFSET = 0.04;
    public static double GLOBAL_PIVOT_OFFSET = 0.0125;
    public static double WRIST_OFFSET = 0.00;

    // Transfer
    public static double liftTransfer = 0;
    public static double pivotTransfer = 0.17;
    public static double wristTransfer = 0.26 + WRIST_OFFSET;

    // Spec Score
    public static double liftSpecScore = 350;
    public static double pivotSpecScore = 0.31 + GLOBAL_PIVOT_OFFSET;
    public static double wristSpecScore = 0.57 + WRIST_OFFSET;

    // Bucket Score
    public static double liftBucketScore = 825;
    public static double liftLowBucketScore = 155;
    public static double pivotBucketScore = 0.6 + GLOBAL_PIVOT_OFFSET;
    public static double wristBucketScore = 0.66 + WRIST_OFFSET; // 0.62

    // Spec Intake
    public static double liftSpecIntake = 200;
    public static double pivotSpecIntake = 0.9 + GLOBAL_PIVOT_OFFSET;
    public static double wristSpecIntake = 0.72 + WRIST_OFFSET;

    // Auto Spec Score
    public static double autoLiftSpecIntake = 350;
    public static double autoPivotSpecIntake = 0.31 + GLOBAL_PIVOT_OFFSET;
    public static double autoWristSpecIntake = 0.57 + WRIST_OFFSET;

    // Claw
    public static double clawOpen = 0.55;
    public static double clawClose = 0.4;
    public static double specOpen = 0.7;

    public PIDCoefficients coef;
    public PIDFController pid;

    public static double kP = 0.00015;
    public static double kI = 0;
    public static double kD = 0.000000015;
    public static double kF = 0.05;
    public static double kvf = 0.000001;

    private double newPower = 0.0;

    public enum ManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public enum HangManualControl {
        IDLE,
        ACTIVATED,
        USING,
        LET_GO
    }

    public ManualControl state = ManualControl.IDLE;

    public HangManualControl hangmanstate = HangManualControl.IDLE;

    // Lift
    public DcMotorEx lift, lift2;


    // PTO
    public DcMotorEx leftBack, rightBack;

    // Arm
    public Servo wrist, left, right, grab;

    // Hang
    public Servo leftPTO, leftRack, rightPTO, rightRack;

    // Auto Grab
    DigitalChannel beamBrake;
    public FArm(HardwareMap hardwareMap) {
        // Arm
        wrist = hardwareMap.get(Servo.class, "wrist");
        left = hardwareMap.get(Servo.class, "leftPivot");
        right = hardwareMap.get(Servo.class, "rightPivot");
        grab = hardwareMap.get(Servo.class, "grab");

        // Lift
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hang
        leftPTO = hardwareMap.get(Servo.class,"leftPTO");
        rightPTO = hardwareMap.get(Servo.class,"rightPTO");
        leftRack = hardwareMap.get(Servo.class, "leftRack");
        rightRack = hardwareMap.get(Servo.class, "rightRack");

        // Auto Grab
        beamBrake = hardwareMap.get(DigitalChannel.class, "beam");
        beamBrake.setMode(DigitalChannel.Mode.INPUT);

        resetEncoder();

        coef = new PIDCoefficients(kP, kI, kD);
        pid = new PIDFController(coef, 0, 0,0, (t, x, v)-> 0.0);

        PID_ENABLED = true;
    }

    double ff = 0;

    public void update() {
        pid.setTargetPosition(targetPosition);

        if (PID_ENABLED) {
            newPower = this.pid.update(lift.getCurrentPosition(), lift.getVelocity());

            if (lift.getVelocity() > 0) {
                ff = Math.abs(lift.getVelocity()) * kvf;
            }

            lift.setPower(newPower + kF + ff);
            lift2.setPower(newPower + kF + ff);
        }
    }

    public void updatePID() {
        pid.updatePIDCoef(kP, 0, kD);
    }

    public void updatePID(double kP, double kI, double kD) {
        pid.updatePIDCoef(kP, kI, kD);
    }

    public void resetEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;
    }

    public boolean isMotorBusy() {
        return Math.abs(pid.getLastError()) > tolerance;
    }

    public void setSpecScore() {
        targetPosition = liftSpecScore; // Does not enable pid
        setPivot(pivotSpecScore);
        wrist.setPosition(wristSpecScore);
    }

    public void setAutoSpecScore() {
        targetPosition = autoLiftSpecIntake; // Does not enable pid
        setPivot(autoPivotSpecIntake);
        wrist.setPosition(autoWristSpecIntake);
    }

    public void setTransfer() {
//        targetPosition = liftTransfer; // Does not enable pid
        setPivot(pivotTransfer);
        wrist.setPosition(wristTransfer);
    }

    public boolean hasSpec() {
        return !beamBrake.getState();
    }

    public void setBucketScore() {  // default is high, when true it is low bucket
        setBucketScore(false);
    }

    public void setBucketScore(boolean low) {
        if (low) targetPosition = liftLowBucketScore;
        else targetPosition = liftBucketScore;
         // Does not enable pid

        setPivot(pivotBucketScore);
        wrist.setPosition(wristBucketScore);
    }

    public boolean isClawOpen() {
        return ActionUtil.compareDouble(grab.getPosition(), clawOpen) || ActionUtil.compareDouble(grab.getPosition(), specOpen);

    }

    public void setSpecIntake() {
        targetPosition = liftSpecIntake; // Does not enable pid
        setPivot(pivotSpecIntake);
        wrist.setPosition(wristSpecIntake);
    }

    public void drop() {
        grab.setPosition(clawOpen);
    }

    public void close() {
        grab.setPosition(clawClose);
    }

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
                lift2.setPower(joystickInput);
                if (Math.abs(joystickInput) < joystickDeadzone) state = FArm.ManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = lift.getCurrentPosition();
                PID_ENABLED = true;
                state = FArm.ManualControl.IDLE;
                break;
        }
    }
//TODO: double check if the motors are going the right way
    public void hangManualControl(double joystickInput) {
        switch (hangmanstate) {
            case IDLE:
                if (Math.abs(joystickInput) > joystickDeadzone)
                    hangmanstate = HangManualControl.ACTIVATED;
                break;
            case ACTIVATED:
                PID_ENABLED = false;

                hangmanstate = HangManualControl.USING;
                break;
            case USING:
                lift.setPower(joystickInput);
                lift2.setPower(joystickInput);
                leftBack.setPower(joystickInput);
                rightBack.setPower(joystickInput);
                if (Math.abs(joystickInput) < joystickDeadzone) hangmanstate = HangManualControl.LET_GO;
                break;
            case LET_GO:
                targetPosition = lift.getCurrentPosition();
                PID_ENABLED = false;
                hangmanstate = HangManualControl.IDLE;
                break;
        }
    }
    //yeah i got no clue if this shit will work i just copied and pasted the old thing and made it a new thing

    public void setPivot(double position) {
        left.setPosition(position);
        right.setPosition(1 - position + PIVOT_OFFSET);
    }
}
