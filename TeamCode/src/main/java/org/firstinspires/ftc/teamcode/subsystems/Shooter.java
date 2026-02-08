package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Shooter extends Hardware {
    // Transfer transfer = new Transfer(hardwareMap);
    public Shooter(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void shoot(double velocity) {
        shooter.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public void update(boolean shoot, int velocity) {
        if (shoot) {
            shooter.setVelocity(velocity);
            shooter2.setVelocity(velocity);
            trigger.setPosition(Constants.TRIGGER_OPEN);

            // Fixing misplaced trigger that Zach won;t admit
//            intake.setPower(-1);
//            new SleepAction(1);
//            intake.setPower(0);

            if (shooter.getVelocity() >= velocity) {

                shooter.setPower(1);
                shooter2.setPower(1);
                transfer.setPower(Constants.TRANSFER_SPEED);
                intake.setPower(1);
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
        shooter2.setVelocity(0);
        trigger.setPosition(Constants.TRIGGER_CLOSE);
    }

    public Action shootAction() {
        return new SequentialAction(
                new InstantAction(() -> shoot(Constants.SHOOTER_VELOCITY)),
                new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_OPEN)),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        return shooter.getVelocity() < Constants.SHOOTER_VELOCITY;
                    }
                },
                new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                new InstantAction(()-> intake.setPower(1)),
                new SleepAction(2),
                new InstantAction(() -> shoot(0)),
                new InstantAction(()-> intake.setPower(0)),
                new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_CLOSE)));
    }
    public Action shootAction2() {
        return new SequentialAction(
                new InstantAction(()-> hood.setPosition(.36)),
                new InstantAction(() -> shoot(Constants.SHOOTER_VELOCITY)),
                new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_OPEN)),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        return shooter.getVelocity() < Constants.SHOOTER_VELOCITY;
                    }
                },
                new ParallelAction(new SequentialAction(
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(2),
                        new InstantAction(() -> shoot(0)),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(() -> trigger.setPosition(Constants.TRIGGER_CLOSE))),

                new SequentialAction(
                        new SleepAction(.2),
                        new InstantAction(()-> hood.setPosition(Constants.HOOD_MIDDLE_LIMIT))
                )));
    }
}
