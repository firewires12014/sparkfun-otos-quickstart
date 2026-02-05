package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Hood extends Hardware {

    public Hood(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    private double targetPosition = Constants.HOOD_LOWER_LIMIT;

    /** Set hood servo to an explicit position (0..1). */
    public void setPosition(double position) {
        targetPosition = position;
        hood.setPosition(targetPosition);
    }

    public double lerp (double distance) {
        double hoodPosition = -.000578233 * Math.pow(distance,2) + 0.0724618 * distance -2.03481;
        if (hoodPosition < 0) hoodPosition = 0;
        else if (hoodPosition > Constants.HOOD_UPPER_LIMIT) hoodPosition = Constants.HOOD_UPPER_LIMIT;
        return hoodPosition;
    }

    /** Move hood to the configured upper limit. */
    public void up() {
        setPosition(Constants.HOOD_MIDDLE_LIMIT);
    }

    /** Move hood to the configured lower limit. */
    public void down() {
        setPosition(Constants.HOOD_LOWER_LIMIT);
    }

    /** Adjust hood position by increment. */
    public void moveHood(double increment) {
        targetPosition += increment;
        // Clamp between lower (0.0) and upper (0.3)
        // Ensure values are correct logic: up is positive?
        // Assuming Limits: Lower=0.0, Upper=0.3.
        if (targetPosition > Constants.HOOD_UPPER_LIMIT)
            targetPosition = Constants.HOOD_UPPER_LIMIT;
        if (targetPosition < Constants.HOOD_LOWER_LIMIT)
            targetPosition = Constants.HOOD_LOWER_LIMIT;
        hood.setPosition(targetPosition);
    }

    public double getPosition() {
        return targetPosition;
    }
}
