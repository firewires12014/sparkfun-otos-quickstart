package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.util.PDFL;

@Config
public class Shooter extends Hardware {
     //Transfer transfer = new Transfer(hardwareMap);

    PDFL pid;
    public static double kp = 0.01;
    public static double kd = 0.00008;
    public static double kf = .0004;
    public static double kv;

    public static double BLUE_SHOT_VELOCITY = 1200;
    public static double BLUE_HOOD_POSITION = 0.11;



    public Shooter(HardwareMap hardwareMap) {
        super(hardwareMap);
        pid = new PDFL(kp, kd, 0, 0);
        pidUpdate();
    }



    public void shoot(double velocity) {
//        shooter.setVelocity(velocity);
//        shooter2.setVelocity(velocity);
        setVelocity(velocity);
    }

    public void setVelocity (double velocity) {
        double power = pid.run(velocity - shooter.getVelocity());
        shooter.setPower(power + kf * velocity);
        shooter2.setPower(power + kf * velocity);
    }

    public void pidUpdate () {
        pid.updateConstants(kp, kd, 0, 0);
    }

    public void update(boolean shoot, int velocity) {
        if (shoot) {
            shoot(velocity);

//            intake.setPower(-1);
//            new SleepAction(1);
//            intake.setPower(0);

            if (shooter.getVelocity() >= velocity && velocity != 0) {
                //gate.setPosition(Constants.TRIGGER_OPEN);
                transfer.setPower(Constants.TRANSFER_SPEED);
                intake.setPower(1);
            }
        } else {
            shooter.setPower(0);
            shooter2.setPower(0);
            gate.setPosition(Constants.TRIGGER_CLOSE);
        }
    }

    public void reverse() {
        shooter.setPower(-0.5);
        gate.setPosition(Constants.TRIGGER_CLOSE);
    }

    public void stop() {
        shooter.setVelocity(0);
        shooter2.setVelocity(0);
        gate.setPosition(Constants.TRIGGER_CLOSE);
    }

    public Action shootAction() {
        return new SequentialAction(
                new InstantAction(() -> shoot(Constants.SHOOTER_VELOCITY)),
                new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
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
                new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE)));
    }

    public Action shootActionRed() {
        final boolean[] done = {false};

        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(()-> hood.setPosition(.15)),
                        new InstantAction(()-> transfer.setPower(-1)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                return (shooter.getVelocity() < 1175 ||
                                        shooter2.getVelocity() < 1175);
                            }
                        },
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(1.75),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE)),
                        new InstantAction(() -> done[0] = true)
                ),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (done[0]) {
                            //setVelocity(0);
                            return false;
                        }
//                        setVelocity(1100);
                        packet.put("Shooter Velocity:", shooter.getVelocity());
                        return true;
                    }
                }
        );
    }
//

    public Action shootActionBlue() {
        final boolean[] done = {false};

        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(()-> hood.setPosition(BLUE_HOOD_POSITION)),
                        new InstantAction(()-> transfer.setPower(-1)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                return (shooter.getVelocity() < 1200 ||
                                        shooter2.getVelocity() < 1200);
                            }
                        },
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(1.75),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE)),
                        new InstantAction(() -> done[0] = true)
                ),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (done[0]) {
                          //  setVelocity(0);
                            return false;
                        }
                        //setVelocity(BLUE_SHOT_VELOCITY);
                        packet.put("Shooter Velocity:", shooter.getVelocity());
                        return true;
                    }
                }
        );
    }

    public Action shootActionBlue2() {
        final boolean[] done = {false};

        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(()-> hood.setPosition(.1)),
                        new InstantAction(()-> transfer.setPower(-1)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                return (shooter.getVelocity() < 1200 ||
                                        shooter2.getVelocity() < 1200);
                            }
                        },
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(1.75),
                        new InstantAction(() -> shoot(0)),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE)),
                        new InstantAction(() -> done[0] = true)
                ),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (done[0]) {
                            setVelocity(0);
                            return false;
                        }
                        setVelocity(1200);
                        packet.put("Shooter Velocity:", shooter.getVelocity());
                        return true;
                    }
                }
        );
    }

    public Action shootActionBlueFar() {
        final boolean[] done = {false};

        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(()-> hood.setPosition(.22)),
                        new InstantAction(()-> transfer.setPower(-1)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                return (shooter.getVelocity() < 1600 ||
                                        shooter2.getVelocity() < 1600);
                            }
                        },
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(1.75),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE)),
                        new InstantAction(() -> done[0] = true)
                ),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (done[0]) {
                           // setVelocity(0);
                            return false;
                        }
                        setVelocity(1600);
                        packet.put("Shooter Velocity:", shooter.getVelocity());
                        return true;
                    }
                }
        );
    }

    public Action shootActionBlueFarModified() {
        return new SequentialAction(
                new InstantAction(()-> turret.setPosition(.415)),
                new InstantAction(()-> hood.setPosition(.24)),
                new InstantAction(() -> shoot(1725)),
                new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                new SleepAction(3),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        return (shooter.getVelocity() < Constants.SHOOTER_VELOCITY &&
                                shooter2.getVelocity() < Constants.SHOOTER_VELOCITY);
                    }
                },
                new ParallelAction(new SequentialAction(
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(2),
                        new InstantAction(() -> shoot(0)),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE))),

                        new SequentialAction(
                                new SleepAction(.2),
                                new InstantAction(()-> hood.setPosition(.24))
                        )));
    }

    public Action shootActionRedFar() {
        return new SequentialAction(
                new InstantAction(()-> turret.setPosition(.365)),
//                new SleepAction(2),
                new InstantAction(()-> hood.setPosition(.17)),
                new InstantAction(() -> shoot(1800)),
                new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        return (shooter.getVelocity() < Constants.SHOOTER_VELOCITY &&
                                shooter2.getVelocity() < Constants.SHOOTER_VELOCITY);
                    }
                },
                new ParallelAction(new SequentialAction(
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(2),
                        new InstantAction(() -> shoot(0)),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE))),

                        new SequentialAction(
                                new SleepAction(.2),
                                new InstantAction(()-> hood.setPosition(.24))
                        )));
    }

    public Action shootActionRedFar2() {
        return new SequentialAction(
                new InstantAction(()-> turret.setPosition(.483)),
                new InstantAction(()-> hood.setPosition(.13)),
                new InstantAction(() -> shoot(1920)),
                new InstantAction(() -> gate.setPosition(Constants.TRIGGER_OPEN)),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        return (shooter.getVelocity() < Constants.SHOOTER_VELOCITY &&
                                shooter2.getVelocity() < Constants.SHOOTER_VELOCITY);
                    }
                },
                new ParallelAction(new SequentialAction(
                        new InstantAction(() -> transfer.setPower(Constants.TRANSFER_SPEED)),
                        new InstantAction(()-> intake.setPower(1)),
                        new SleepAction(2),
                        new InstantAction(() -> shoot(0)),
                        new InstantAction(()-> intake.setPower(0)),
                        new InstantAction(()-> transfer.setPower(0)),
                        new InstantAction(() -> gate.setPosition(Constants.TRIGGER_CLOSE))),

                        new SequentialAction(
                                new SleepAction(.2),
                                new InstantAction(()-> hood.setPosition(.24))
                        )));
    }
}
