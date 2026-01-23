package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.TRANSFER_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_OPEN;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Transfer extends Hardware {

    public Transfer(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void run() {
        transfer.setPower(TRANSFER_SPEED);
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

    public void triggerOpen() { trigger.setPosition(TRIGGER_OPEN); }
    public void triggerClose() { trigger.setPosition(TRIGGER_CLOSE);}
}
