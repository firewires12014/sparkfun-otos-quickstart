package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "02. LimelightTurretTest", group = "TeleOp")
public class LimelightTurret extends OpMode {

    // Servo position bounds (standard servo: 0.0 = full one way, 1.0 = full other way).
    private static final double SERVO_MIN = 0.034
    private static final double SERVO_MAX = 0.495;
    private static final double SERVO_CENTER = 0.68;

    public static double hardLeft = 0.34; //-45
    public static double hardRight = .68; //47
    public static double middle = .495; // 0

    // Proportional gain: how much the servo moves per degree of horizontal error per loop.
    // This is a high-speed servo, so KP is intentionally tiny.
    private static final double KP = 0.0001;

    // Hard cap on how far the servo can move in a single loop. Prevents lurching even
    // when tx is huge. With ~50Hz loop rate, 0.0008 ≈ 0.04 servo units / second max.
    private static final double MAX_STEP_PER_LOOP = 0.0012;

    // Don't bother adjusting if within this many degrees — keeps the servo from buzzing.
    private static final double DEADBAND_DEGREES = 0.5;

    // Flip if the turret rotates the wrong way relative to the camera.
    private static final double DIRECTION = 1.0;

    // Pipeline 0 is configured as a QR / barcode pipeline. Lock onto this QR data string.
    private static final int TARGET_PIPELINE = 0;
    private static final String TARGET_QR_DATA = "24";

    private Servo turret;
    private Limelight3A limelight;
    private double servoPosition = SERVO_CENTER;

    @Override
    public void init() {
        turret = hardwareMap.get(Servo.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servoPosition = SERVO_CENTER;
        turret.setPosition(servoPosition);

        limelight.pipelineSwitch(TARGET_PIPELINE);

        telemetry.addData(">", "Robot Ready. Press Play.");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();

            if (Math.abs(tx) > DEADBAND_DEGREES) {
                double step = DIRECTION * KP * tx;
                step = Range.clip(step, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
                servoPosition += step;
                servoPosition = Range.clip(servoPosition, SERVO_MIN, SERVO_MAX);
                turret.setPosition(servoPosition);
            }

            telemetry.addData("Tx", tx);
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Servo", "%.3f", servoPosition);
        } else {
            telemetry.addLine("None found");
            telemetry.addData("Servo", "%.3f", servoPosition);
        }
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
