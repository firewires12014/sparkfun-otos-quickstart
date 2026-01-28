// Java
package org.firstinspires.ftc.teamcode.subsystems;

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
    private int zeroTicks = 0;
    private double lastError = 0;
    private boolean manual = false;
    private double manualPower = 0.0;

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

    // Pass in the robot pose from TeleOp to avoid drive dependency
    public void update(Pose2d robotPose, Vector2d target) {
        if (manual) return;

        double targetAngle = 0.0;
        if (target != null && robotPose != null) {
            double dx = target.x - robotPose.position.x;
            double dy = target.y - robotPose.position.y;
            double headingRad = robotPose.heading.toDouble(); // Rotation2d -> radians
            targetAngle = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - headingRad);
        }

        double clampedTarget = Range.clip(targetAngle, Constants.MIN_RAD, Constants.MAX_RAD);
        double currentPos = (turretEncoder.getCurrentPosition() - zeroTicks) * Constants.TICKS_TO_RADIANS;

        double error = AngleUnit.normalizeRadians(clampedTarget - currentPos);
        double derivative = error - lastError;
        double power = (error * 1.5) + (derivative * 5.0);
        lastError = error;

        if (currentPos > (Constants.MAX_RAD - Constants.BUFFER) && power > 0.1) power = 0.1;
        if (currentPos < (Constants.MIN_RAD + Constants.BUFFER) && power < -0.1) power = -0.1;

        turret.setPower(Range.clip(power, -1.0, 1.0));
    }
}