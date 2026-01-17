package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Intake extends Hardware {

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void in() {
        intake.setPower(1);
    }

    public void out() {
        intake.setPower(-1);
    }

    public void stop() {
        intake.setPower(0);
    }
}
