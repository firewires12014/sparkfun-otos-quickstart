package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Turret extends Hardware {

    /** Encoder counts per turret revolution (CPR) after the external gearing. */
    public static double TICKS_PER_REV = 8192.0;

    /** Gear ratio from encoder shaft to turret (encoderRev * GEAR_RATIO = turretRev). */
    public static double GEAR_RATIO = 1.0;

    /** Hard limits relative to straight-forward (degrees). */
    public static double LIMIT_LEFT_DEG = 90.0;
    public static double LIMIT_RIGHT_DEG = 90.0;

    /** Small power used for simple rotateLeft/rotateRight helpers. */
    public static double DEFAULT_POWER = 1.0;

    /** If encoder direction is inverted, set to -1. */
    public static int ENCODER_SIGN = 1;

    /** If the servo power direction is inverted (left/right swapped), set to -1. */
    public static int SERVO_SIGN = 1;

    /** Auto-point controller tuning (simple P). */
    public static double AUTO_KP = 0.015; // power per degree of error
    public static double AUTO_MAX_POWER = 0.6;
    public static double AUTO_DEADBAND_DEG = 2.0;

    /** Desired field-relative turret headings for auto-point presets (degrees). */
    public static double FIELD_TARGET_A_DEG = -72.0;
    public static double FIELD_TARGET_B_DEG = 72.0;

    private int zeroTicks = 0;
    private boolean initialized = false;
    private Double targetAngleDeg = null;

    private Double lastRobotHeadingDeg = null;
    private Double lastFieldTargetDeg = null;
    private Double lastRelativeTargetDeg = null;

    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap);

        // The encoder is read through a DcMotorEx (see Hardware.java). Ensure it is usable as a sensor.
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Assume we start pointed forward.
        zeroForward();
    }

    /** Call once at init (or any time) when turret is pointed straight forward. */
    public void zeroForward() {
        zeroTicks = turretEncoder.getCurrentPosition();
        initialized = true;
    }

    /** Current turret angle (radians) relative to the forward-zero. Positive is "left" by convention. */
    public double getAngleRad() {
        if (!initialized) zeroForward();
        int ticks = turretEncoder.getCurrentPosition();
        int delta = (ticks - zeroTicks) * ENCODER_SIGN;
        return (delta / TICKS_PER_REV) * 2.0 * Math.PI / GEAR_RATIO;
    }

    public double getAngleDeg() {
        return Math.toDegrees(getAngleRad());
    }

    public double getMinAngleRad() {
        return -Math.toRadians(LIMIT_RIGHT_DEG);
    }

    public double getMaxAngleRad() {
        return Math.toRadians(LIMIT_LEFT_DEG);
    }

    /** Returns true if turret is within the configured hard limits. */
    public boolean isWithinLimits() {
        double a = getAngleRad();
        return a >= getMinAngleRad() && a <= getMaxAngleRad();
    }

    /**
     * Sets turret servo power, enforcing hard limits based on encoder.
     * Positive power attempts to rotate left; negative attempts to rotate right.
     */
    public void setPowerLimited(double requestedPower) {
        if (!initialized) zeroForward();

        double min = getMinAngleRad();
        double max = getMaxAngleRad();
        double angle = getAngleRad();

        double p = clip(requestedPower, -1.0, 1.0);

        // Stop if pushing farther past a limit.
        if ((angle >= max && p > 0) || (angle <= min && p < 0)) {
            p = 0.0;
        }

        turret.setPower(p * SERVO_SIGN);
    }

    public void rotateLeft() {
        setPowerLimited(Math.abs(DEFAULT_POWER));
    }

    public void rotateRight() {
        setPowerLimited(-Math.abs(DEFAULT_POWER));
    }

    public void stop() {
        turret.setPower(0);
    }

    /** Convenience helpers if you want explicit checks. */
    public boolean atLeftLimit() {
        return getAngleRad() >= getMaxAngleRad();
    }

    public boolean atRightLimit() {
        return getAngleRad() <= getMinAngleRad();
    }

    /** Set a new auto-point target angle (degrees), relative to forward-zero. */
    public void setTargetAngleDeg(double angleDeg) {
        targetAngleDeg = angleDeg;
    }

    /** Clears the auto-point target; turret will not drive itself. */
    public void clearTarget() {
        targetAngleDeg = null;
    }

    /** Returns the current auto-point target angle in degrees, or null if none. */
    public Double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    /**
     * Auto-point update loop. Call periodically.
     * Uses a simple proportional controller and still enforces hard limits.
     */
    public void updateAutoPoint() {
        if (targetAngleDeg == null) return;

        double errorDeg = targetAngleDeg - getAngleDeg();
        if (Math.abs(errorDeg) <= AUTO_DEADBAND_DEG) {
            stop();
            return;
        }

        double power = clip(errorDeg * AUTO_KP, -AUTO_MAX_POWER, AUTO_MAX_POWER);
        setPowerLimited(power);
    }

    /** Helper: turret aims at either (-72,-72) or (-72,72) -> equivalent to -72° or +72° from forward. */
    public void pointToMinus72Minus72() {
        setTargetAngleDeg(-72.0);
    }

    /** Helper: turret aims at either (-72,-72) or (-72,72) -> equivalent to -72° or +72° from forward. */
    public void pointToMinus72Plus72() {
        setTargetAngleDeg(72.0);
    }

    /**
     * Convert a field-relative desired heading into a turret-relative target (degrees)
     * using the robot heading from Road Runner pose.
     */
    public double fieldToTurretRelativeDeg(Pose2d robotPose, double desiredFieldHeadingDeg) {
        double robotHeadingDeg = Math.toDegrees(robotPose.heading.toDouble());
        double rel = wrapDeg(desiredFieldHeadingDeg - robotHeadingDeg);
        return clipToTurretLimitsDeg(rel);
    }

    /**
     * Auto-point the turret to a field-relative heading using Road Runner pose.
     * This sets the internal target and records debug values.
     */
    public void setTargetFieldHeadingDeg(Pose2d robotPose, double desiredFieldHeadingDeg) {
        double robotHeadingDeg = Math.toDegrees(robotPose.heading.toDouble());
        double rel = fieldToTurretRelativeDeg(robotPose, desiredFieldHeadingDeg);

        lastRobotHeadingDeg = robotHeadingDeg;
        lastFieldTargetDeg = desiredFieldHeadingDeg;
        lastRelativeTargetDeg = rel;

        setTargetAngleDeg(rel);
    }

    /** Field-relative preset A using Road Runner pose. */
    public void pointToFieldTargetA(Pose2d robotPose) {
        setTargetFieldHeadingDeg(robotPose, FIELD_TARGET_A_DEG);
    }

    /** Field-relative preset B using Road Runner pose. */
    public void pointToFieldTargetB(Pose2d robotPose) {
        setTargetFieldHeadingDeg(robotPose, FIELD_TARGET_B_DEG);
    }

    /**
     * Debug helper: adds turret encoder/angle/limit info to telemetry.
     * Works with both normal Telemetry and MultipleTelemetry.
     */
    public Telemetry addTelemetry(Telemetry telemetry) {
        if (telemetry == null) return null;

        telemetry.addData("TurretAngleDeg", "%.1f", getAngleDeg());
        telemetry.addData("TurretTargetDeg", targetAngleDeg == null ? "none" : String.format("%.1f", targetAngleDeg));
        telemetry.addData("TurretErrorDeg", targetAngleDeg == null ? "none" : String.format("%.1f", (targetAngleDeg - getAngleDeg())));

        telemetry.addData("RobotHeadingDeg", lastRobotHeadingDeg == null ? "n/a" : String.format("%.1f", lastRobotHeadingDeg));
        telemetry.addData("TurretFieldTargetDeg", lastFieldTargetDeg == null ? "n/a" : String.format("%.1f", lastFieldTargetDeg));
        telemetry.addData("TurretRelativeTargetDeg", lastRelativeTargetDeg == null ? "n/a" : String.format("%.1f", lastRelativeTargetDeg));

        telemetry.addData("TurretLeftLimitDeg", "%.1f", LIMIT_LEFT_DEG);
        telemetry.addData("TurretRightLimitDeg", "%.1f", LIMIT_RIGHT_DEG);
        telemetry.addData("TurretAtLeftLimit", atLeftLimit());
        telemetry.addData("TurretAtRightLimit", atRightLimit());
        return telemetry;
    }

    /**
     * Convenience overload to create a MultipleTelemetry instance and add turret data to it.
     * Example: turret.addTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
     */
    public Telemetry addTelemetry(Telemetry primary, Telemetry secondary) {
        if (primary == null) return null;
        Telemetry t = (secondary == null) ? primary : new MultipleTelemetry(primary, secondary);
        return addTelemetry(t);
    }

    /** Normalize degrees to [-180, 180). */
    private static double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    /** Clips a desired turret relative angle to the configured hard limits. */
    private double clipToTurretLimitsDeg(double relativeDeg) {
        return clip(relativeDeg, -LIMIT_RIGHT_DEG, LIMIT_LEFT_DEG);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
