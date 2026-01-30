package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Turret extends Hardware {
    private static final String TAG = "Turret";

    // Minimum continuous-rotation servo power to reliably move the turret (tunable)
    public static double MIN_MOVE_POWER = 0.5;

    private int zeroTicks = 0;
    private double lastError = 0;
    private boolean manual = false;
    private double manualPower = 0.0;
    // If manualUntilMs > now, manual mode is active only until that timestamp (ms). This
    // allows a short initial kick to overcome stiction then return to closed-loop control.
    private long manualUntilMs = 0;

    // track the turret's world angle (radians). When NaN it will be initialized on first update
    private double worldTurretAngle = Double.NaN;

    // Debug / exported values
    private double robotHeadingDeg = 0.0;
    private double lastTargetAngle = 0.0;
    private double lastClampedTarget = 0.0;
    private double lastCurrentPos = 0.0;
    private double lastErrorVal = 0.0;
    private double lastDerivative = 0.0;
    private double lastPower = 0.0;

    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap);
        // Do not auto-zero the encoder at construction. If the robot isn't physically
        // aligned to robot-forward at startup, auto-zeroing will make the controller
        // assume the turret is already forward and prevent initial movement. Leave
        // zeroTicks at 0 so the first update initializes worldTurretAngle correctly.
        zeroTicks = 0;
    }

    public void zeroForward() {
        zeroTicks = turretEncoder.getCurrentPosition();
    }

    public void setManualPower(double power) {
        manual = true;
        manualPower = Range.clip(power, -1.0, 1.0);
        turret.setPower(manualPower);
    }

    public void stop() {
        manual = false;
        turret.setPower(0.0);
    }

    public double getRobotRelativeDeg() {
        double currentRad = (turretEncoder.getCurrentPosition() - zeroTicks) * Constants.TICKS_TO_RADIANS;
        return Math.toDegrees(currentRad);
    }

    // Exported getters for debug
    public double getRobotHeadingDeg() { return robotHeadingDeg; }
    public double getLastTargetAngle() { return lastTargetAngle; }
    public double getLastClampedTarget() { return lastClampedTarget; }
    public double getLastCurrentPos() { return lastCurrentPos; }
    public double getLastError() { return lastErrorVal; }
    public double getLastDerivative() { return lastDerivative; }
    public double getLastPower() { return lastPower; }

    // Formatted debug string (easy to telemetry or log)
    public String getDebugString() {
        return String.format(
                "headingDeg=%.2f, targetRad=%.3f, clamped=%.3f, pos=%.3f, err=%.3f, der=%.3f, pwr=%.3f",
                robotHeadingDeg, lastTargetAngle, lastClampedTarget, lastCurrentPos, lastErrorVal, lastDerivative, lastPower
        );
    }

    // Pass in the robot pose from TeleOp to avoid drive dependency
    public void update(Pose2d robotPose, Vector2d target) {
        // If we're in manual mode due to a timed kick, maintain manual power until the
        // timeout expires; otherwise fall through into closed-loop control.
        if (manual) {
            long now = System.currentTimeMillis();
            if (now < manualUntilMs) {
                turret.setPower(manualPower);
                // still in timed manual window
                return;
            } else {
                // timed manual window expired; return to closed-loop
                manual = false;
            }
        }

        if (manual) return;

        double targetAngleRobotRel = 0.0; // desired turret angle relative to robot
        if (robotPose != null) {
            double headingRad = robotPose.heading.toDouble(); // Rotation2d -> radians
            // store exported heading in degrees
            robotHeadingDeg = Math.toDegrees(headingRad);

            // current turret robot-relative position
            double currentPos = (turretEncoder.getCurrentPosition() - zeroTicks) * Constants.TICKS_TO_RADIANS;

            // initialize worldTurretAngle on first valid pose update to the turret's current world-facing angle
            if (Double.isNaN(worldTurretAngle)) {
                worldTurretAngle = AngleUnit.normalizeRadians(currentPos + headingRad);
            }

            if (target != null) {
                double dx = target.x - robotPose.position.x;
                double dy = target.y - robotPose.position.y;
                // desired world-facing angle toward the target
                double desiredWorldAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx));
                // update stored world-facing target so we continue to hold it if target later becomes null
                worldTurretAngle = desiredWorldAngle;
            }

            // compute desired turret angle relative to robot so that turret faces worldTurretAngle
            targetAngleRobotRel = AngleUnit.normalizeRadians(worldTurretAngle - headingRad);
            lastTargetAngle = targetAngleRobotRel;

            // proceed with the rest of the controller using targetAngleRobotRel below

            double clampedTarget = Range.clip(targetAngleRobotRel, Constants.MIN_RAD, Constants.MAX_RAD);

            double error = AngleUnit.normalizeRadians(clampedTarget - currentPos);
            double derivative = error - lastError;
            double power = (error * 1.5) + (derivative * 5.0);
            lastError = error;

            if (currentPos > (Constants.MAX_RAD - Constants.BUFFER) && power > 0.1) power = 0.1;
            if (currentPos < (Constants.MIN_RAD + Constants.BUFFER) && power < -0.1) power = -0.1;

            double clippedPower = Range.clip(power, -1.0, 1.0);

            // If controller output is too small to move the continuous-rotation servo
            // but the angular error is meaningful, enforce a minimum move power to
            // overcome static friction. This helps when lastPower is tiny (eg 0.0001)
            // which isn't enough to move the turret.
            double absErr = Math.abs(error);
            if (absErr > Math.toRadians(0.5) && Math.abs(clippedPower) < MIN_MOVE_POWER) {
                double useSign = (Math.abs(power) > 1e-6) ? Math.signum(power) : Math.signum(error);
                clippedPower = Range.clip(useSign * MIN_MOVE_POWER, -1.0, 1.0);
                Log.d(TAG, "Enforcing min move power: " + clippedPower + " errorDeg=" + Math.toDegrees(error));
            }

            turret.setPower(clippedPower);

            // store debug values
            lastClampedTarget = clampedTarget;
            lastCurrentPos = currentPos;
            lastErrorVal = error;
            lastDerivative = derivative;
            lastPower = clippedPower;

            // Log debug info so it appears in logcat (device logs)
            Log.d(TAG, getDebugString());
            return;
        }

        // If robotPose is null, keep previous behavior: reset exported heading to NaN for visibility
        robotHeadingDeg = Double.NaN;
        lastTargetAngle = Double.NaN;
        lastClampedTarget = Double.NaN;
        lastCurrentPos = Double.NaN;
        lastErrorVal = Double.NaN;
        lastDerivative = Double.NaN;
        lastPower = 0.0;
    }

    // Convenience overload to preserve existing callers that pass only the robot pose.
    public void update(Pose2d robotPose) {
        update(robotPose, null);
    }

    // Capture the current robot heading as the world-locked turret target. This causes the turret
    // to move to robot-forward (robot-relative angle 0) and then hold that world heading as the
    // robot rotates.
    public void captureWorldHeadingAsRobotForward(Pose2d robotPose) {
        if (robotPose == null) return;
        double headingRad = robotPose.heading.toDouble();
        // Set the desired world-facing angle equal to the robot's heading. That makes the
        // desired robot-relative turret angle zero (forward) immediately, and the turret will
        // compensate for future robot rotation to keep that world-facing direction.
        worldTurretAngle = AngleUnit.normalizeRadians(headingRad);

        // Ensure we're not stuck in manual mode and run an immediate controller update so the
        // turret begins moving toward robot-forward right away.
        manual = false;
        try {
            update(robotPose, null);
        } catch (Exception e) {
            Log.w(TAG, "captureWorldHeadingAsRobotForward: update threw", e);
        }

        // If closed-loop decided to output essentially zero power but there is a
        // significant position error, the turret might be stuck due to static
        // friction or an encoder mismatch. In that case, apply a short manual 'kick'
        // to get it moving, then return control to closed-loop.
        if (Math.abs(lastPower) < 1e-3 && Math.abs(lastErrorVal) > Math.toRadians(1.0)) {
            double kick = Math.signum(lastErrorVal) * 0.35; // directional kick magnitude
            manual = true;
            manualPower = Range.clip(kick, -1.0, 1.0);
            manualUntilMs = System.currentTimeMillis() + 150; // ms
            turret.setPower(manualPower);
            Log.d(TAG, "Applied manual kick to turret: power=" + manualPower + " errorDeg=" + Math.toDegrees(lastErrorVal));
        }

        Log.d(TAG, "captureWorldHeadingAsRobotForward: worldTurretAngle=" + worldTurretAngle + " lastErrorDeg=" + Math.toDegrees(lastErrorVal) + " lastPower=" + lastPower);
    }

    public void rotateLeft() {
        turret.setPower(-.25);
    }

    public void rotateRight() {
        turret.setPower(.25);
    }

}
