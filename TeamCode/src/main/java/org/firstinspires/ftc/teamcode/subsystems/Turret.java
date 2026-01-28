package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Turret extends Hardware {
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

    public void rotateLeft() {
        setPower(1);
    }

    public void rotateRight() {
        setPower(-1);
    }

    public void stop() {
        turret.setPower(0);
    }

    /**
     * Updates turret aiming toward a world-space target.
     * If {@code target} is null, the turret defaults to 0 rad (forward).
     */
    public void updateTurret(Pose2d robotPose, Vector2d target) {
        if (target == null) {
            updateTurret(robotPose);
            return;
        }

        // 1. Calculate Target Angle
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double targetAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robotPose.getHeading());

        // 2. Clamp target to your 180-degree physical reach
        double clampedTarget = Range.clip(targetAngle, Constants.MIN_RAD, MAX_RAD);

        // 3. Feedback Loop
        double currentPos = turretEncoder.getCurrentPosition() * Constants..TICKS_TO_RADIANS;
        double error = clampedTarget - currentPos;

        // 4. Proportional Gain
        double power = error * 1.5;

        // 5. Soft Limit Safety
        // If we are within the buffer zone of the hard stop, cap the power
        if (currentPos > (Constants.MAX_RAD - Constants.BUFFER) && power > 0.1) power = 0.1;
        if (currentPos < (Constants.MIN_RAD + Constants.BUFFER) && power < -0.1) power = -0.1;

        turretServo.setPower(Range.clip(power, -1.0, 1.0));
    }

    /** Default update that drives turret back to 0 rad (forward). */
    public void updateTurret(Pose2d robotPose) {
        // Clamp (0 rad) to physical reach
        double clampedTarget = Range.clip(0.0, Constants.MIN_RAD, MAX_RAD);

        // Feedback Loop
        double currentPos = turretEncoder.getCurrentPosition() * Constants..TICKS_TO_RADIANS;
        double error = clampedTarget - currentPos;

        // Proportional Gain
        double power = error * 1.5;

        // Soft Limit Safety
        if (currentPos > (Constants.MAX_RAD - Constants.BUFFER) && power > 0.1) power = 0.1;
        if (currentPos < (Constants.MIN_RAD + Constants.BUFFER) && power < -0.1) power = -0.1;

        turretServo.setPower(Range.clip(power, -1.0, 1.0));
    }
}
