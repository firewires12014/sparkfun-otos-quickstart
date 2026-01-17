package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Hood extends Hardware {

    public Hood(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /** Set hood servo to an explicit position (0..1). */
    public void setPosition(double position) {
        hood.setPosition(position);
    }

    /** Move hood to the configured upper limit. */
    public void up() {
        hood.setPosition(Constants.HOOD_UPPER_LIMIT);
    }

    /** Move hood to the configured lower limit. */
    public void down() {
        hood.setPosition(Constants.HOOD_LOWER_LIMIT);
    }
}

