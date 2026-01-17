package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret limit tuning opmode.
 *
 * Usage:
 * 1) Start with the turret aimed straight forward.
 * 2) Press A: zero (set forward reference).
 * 3) Hold LB to drive left, hold RB to drive right.
 * 4) When at the mechanical/wire-safe left stop, press X to capture left limit.
 * 5) When at the mechanical/wire-safe right stop, press B to capture right limit.
 *
 * The captured limits are reported in degrees (relative to forward) so you can copy
 * them into Turret.LIMIT_LEFT_DEG and Turret.LIMIT_RIGHT_DEG.
 */
@TeleOp(name = "Tune Turret Limits", group = "Tuning")
public class TurretLimitTuner extends LinearOpMode {

    // Manual tuning power (CRServo power). Keep low-ish for safety.
    private static final double TUNE_POWER = 0.35;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        Telemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Double capturedLeftDeg = null;
        Double capturedRightDeg = null;

        boolean lastA = false, lastB = false, lastX = false;

        multiTelemetry.addLine("Turret Limit Tuner");
        multiTelemetry.addLine("Start with turret pointing FORWARD.");
        multiTelemetry.addLine("A: zero-forward");
        multiTelemetry.addLine("Hold LB: rotate left, Hold RB: rotate right");
        multiTelemetry.addLine("X: capture LEFT limit, B: capture RIGHT limit");
        multiTelemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Edge-detect buttons
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            boolean aPressed = a && !lastA;
            boolean bPressed = b && !lastB;
            boolean xPressed = x && !lastX;

            lastA = a;
            lastB = b;
            lastX = x;

            // Zero forward
            if (aPressed) {
                turret.zeroForward();
                capturedLeftDeg = null;
                capturedRightDeg = null;
            }

            // Manual drive (bypasses auto-point). Uses power limiting to avoid going past currently-set software limits.
            // For tuning, you may want limits wide open first or temporarily set Turret.LIMIT_* big (e.g., 180).
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                turret.setPowerLimited(Math.abs(TUNE_POWER));
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                turret.setPowerLimited(-Math.abs(TUNE_POWER));
            } else {
                turret.stop();
            }

            // Capture limits
            if (xPressed) {
                capturedLeftDeg = turret.getAngleDeg();
            }
            if (bPressed) {
                capturedRightDeg = turret.getAngleDeg();
            }

            // Telemetry
            turret.addTelemetry(multiTelemetry);

            multiTelemetry.addData("CapturedLeftLimitDeg", capturedLeftDeg == null ? "(press X)" : String.format("%.1f", capturedLeftDeg));
            multiTelemetry.addData("CapturedRightLimitDeg", capturedRightDeg == null ? "(press B)" : String.format("%.1f", capturedRightDeg));

            if (capturedLeftDeg != null && capturedRightDeg != null) {
                multiTelemetry.addLine();
                multiTelemetry.addLine("Copy these into Turret.java:");
                multiTelemetry.addData("LIMIT_LEFT_DEG", String.format("%.1f", Math.abs(capturedLeftDeg)));
                multiTelemetry.addData("LIMIT_RIGHT_DEG", String.format("%.1f", Math.abs(capturedRightDeg)));
            }

            multiTelemetry.update();
            idle();
        }
    }
}

