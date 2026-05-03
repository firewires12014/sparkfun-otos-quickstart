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

@TeleOp(name = "02. LimelightTurretTest", group = "MPE")
public class LimelightTurret extends OpMode {

    // Servo position bounds (standard servo: 0.0 = full one way, 1.0 = full other way).
    public static double SERVO_MIN = 0.034;
    public static double SERVO_CENTER = 0.495;
    public static double SERVO_MAX = 0.68;

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
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL Status", status.getName());
        telemetry.addData("LL Pipeline", status.getPipelineIndex());
        telemetry.addData("LL Temp (C)", status.getTemp());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null) {
            telemetry.addData("Result Valid", llResult.isValid());
            
            if (llResult.isValid()) {
                double tx = llResult.getTx();
                double ty = llResult.getTy();
                double ta = llResult.getTa();

                telemetry.addData("Target", "LOCKED");
                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", ty);
                telemetry.addData("Ta", ta);

                // AprilTag specific debug
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                telemetry.addData("Fiducials count", fiducials.size());
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    telemetry.addData("Fid ID", fr.getFiducialId());
                    telemetry.addData("Fid Family", fr.getFamily());
                }

                if (Math.abs(tx) > DEADBAND_DEGREES) {
                    double step = DIRECTION * KP * tx;
                    step = Range.clip(step, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
                    servoPosition += step;
                    servoPosition = Range.clip(servoPosition, SERVO_MIN, SERVO_MAX);
                    turret.setPosition(servoPosition);
                }
            } else {
                telemetry.addLine("No valid target found");
                // Even if not "valid" (main target), check if any fiducials are seen at all
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    telemetry.addData("Fiducials seen (but invalid)", fiducials.size());
                }
            }
        } else {
            telemetry.addLine("LL Result is NULL");
        }
        
        telemetry.addData("Servo Position", "%.3f", servoPosition);
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
