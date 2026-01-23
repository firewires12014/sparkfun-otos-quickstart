package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Shooter extends Hardware {
//    Transfer transfer = new Transfer(hardwareMap);
    public Shooter(HardwareMap hardwareMap) {
        super(hardwareMap);
    }


    public void shoot (double velocity) {
        shooter.setVelocity(velocity);
    }
}

