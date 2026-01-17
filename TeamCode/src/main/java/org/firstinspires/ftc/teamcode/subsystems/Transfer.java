package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Transfer extends Hardware {

    public Transfer(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void run() {
        transfer.setPower(1);
        // Assuming transfer servos are used for feeding, but in code it's motor
        // If servos need to be controlled, add here
    }

    public void reverse() {
        transfer.setPower(-1);
    }

    public void slow() {
        transfer.setPower(-0.25);
    }

    public void stop() {
        transfer.setPower(0);
    }
}
