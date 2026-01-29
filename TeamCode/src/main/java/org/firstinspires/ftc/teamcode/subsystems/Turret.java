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

    private int zeroTicks = 0;
    private double lastError = 0;
    private boolean manual = false;
    private double manualPower = 0.0;

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
        zeroForward();
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
        if (manual) return;

        double targetAngle = 0.0;
        if (target != null && robotPose != null) {
            double dx = target.x - robotPose.position.x;
            double dy = target.y - robotPose.position.y;
            double headingRad = robotPose.heading.toDouble(); // Rotation2d -> radians
            // store exported heading in degrees
            robotHeadingDeg = Math.toDegrees(headingRad);
            targetAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - headingRad);
            lastTargetAngle = targetAngle;
        } else {
            // If pose is null reset exported heading to NaN for visibility
            robotHeadingDeg = robotPose != null ? robotHeadingDeg : Double.NaN;
            lastTargetAngle = Double.NaN;
        }

        double clampedTarget = Range.clip(targetAngle, Constants.MIN_RAD, Constants.MAX_RAD);
        double currentPos = (turretEncoder.getCurrentPosition() - zeroTicks) * Constants.TICKS_TO_RADIANS;

        double error = AngleUnit.normalizeRadians(clampedTarget - currentPos);
        double derivative = error - lastError;
        double power = (error * 1.5) + (derivative * 5.0);
        lastError = error;

        if (currentPos > (Constants.MAX_RAD - Constants.BUFFER) && power > 0.1) power = 0.1;
        if (currentPos < (Constants.MIN_RAD + Constants.BUFFER) && power < -0.1) power = -0.1;

        double clippedPower = Range.clip(power, -1.0, 1.0);
        turret.setPower(clippedPower);

        // store debug values
        lastClampedTarget = clampedTarget;
        lastCurrentPos = currentPos;
        lastErrorVal = error;
        lastDerivative = derivative;
        lastPower = clippedPower;

        // Log debug info so it appears in logcat (device logs)
        Log.d(TAG, getDebugString());
    }

    public void rotateLeft() {
        turret.setPower(-.25);
    }

    public void rotateRight() {
        turret.setPower(.25);
    }

}
