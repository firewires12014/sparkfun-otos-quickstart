package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Lift extends Hardware {

    public Lift(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void up() {
        lift1.setPosition(Constants.LIFT1_UP);
        lift2.setPosition(Constants.LIFT2_UP);
    }

    public void down() {
        lift1.setPosition(Constants.LIFT1_DOWN);
        lift2.setPosition(Constants.LIFT2_DOWN);
    }
}
