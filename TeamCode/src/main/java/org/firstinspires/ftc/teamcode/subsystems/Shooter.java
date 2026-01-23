package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Constants;
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
            shooter.setVelocity(Constants.SHOOTER_VELOCITY);
            trigger.setPosition(Constants.TRIGGER_OPEN);

            if (shooter.getVelocity() >= Constants.SHOOTER_VELOCITY) {
                intake.setPower(1);
                transfer.setPower(Constants.TRANSFER_SPEED);
            }
        } else {
            stop();
        }
    }

    public void reverse() {
        shooter.setPower(-0.5);
        trigger.setPosition(Constants.TRIGGER_CLOSE);
    }

    public void stop() {
        shooter.setVelocity(0);
        trigger.setPosition(Constants.TRIGGER_CLOSE);
    }

    public Action shootAction() {
        return new SequentialAction(
                new InstantAction(() -> shoot(Constants.SHOOTER_VELOCITY)),
                new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_OPEN)),
                new SleepAction(2),
                new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                new SleepAction(2),
                new InstantAction(() -> shoot(0)),
                new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_CLOSE)));
    }
}
