package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Shooter extends Hardware {
    // Transfer transfer = new Transfer(hardwareMap);
    public Shooter(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void shoot(double velocity) {
        shooter.setVelocity(velocity);
    }

    public void update(boolean shoot) {
        if (shoot) {
            shooter.setVelocity(org.firstinspires.ftc.teamcode.Constants.SHOOTER_VELOCITY);
            trigger.setPosition(org.firstinspires.ftc.teamcode.Constants.TRIGGER_OPEN);

            if (shooter.getVelocity() >= org.firstinspires.ftc.teamcode.Constants.SHOOTER_VELOCITY) {
                intake.setPower(1);
                transfer.setPower(org.firstinspires.ftc.teamcode.Constants.TRANSFER_SPEED);
            }
        } else {
            shooter.setVelocity(0);
            trigger.setPosition(org.firstinspires.ftc.teamcode.Constants.TRIGGER_CLOSE);
        }
    }
}
