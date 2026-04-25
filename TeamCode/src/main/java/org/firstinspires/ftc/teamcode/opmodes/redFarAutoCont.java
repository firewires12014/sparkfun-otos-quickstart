package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.AutoTelopSaveState;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "redFarAutoCont")
public final class redFarAutoCont extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        int xOffset = Constants.BLUE_CLOSE_X_OFFSET;
        int yOffset = Constants.BLUE_CLOSE_Y_OFFSET;

        Pose2d startPose = new Pose2d(63, 15, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        Action cycle1 = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(180))
                //                                         Mess with this v       robot angle             path angle
                .splineToLinearHeading(new Pose2d(new Vector2d(32, 40), Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(34, 70))
                .strafeTo(new Vector2d(34,60 ))
                .strafeTo(new Vector2d(34, 70))
                .strafeTo(new Vector2d(63, 15))
                .build();

        Action cycle2 = new ParallelAction(
                drive.actionBuilder(new Pose2d(63, 15, Math.toRadians(90)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 60), Math.toRadians(90)), Math.toRadians(90), drive.defaultVelConstraintSlow) // <---  (too hp area)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(90)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(300), drive.defaultVelConstraintSlow) // (forward to hp area again)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(270)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(60), drive.defaultVelConstraintSlow)
                        // Go back to shoot
                        .waitSeconds(.5) // wait however long it takes to intake
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 15), Math.toRadians(90)), Math.toRadians(90)) // back to shoot

                        .build(),
                new SequentialAction(
                        // new SleepAction(2), // time till end of second spline
                        //  new InstantAction(intake::out),
                        new SleepAction(0), // as need to spit out?
                        new InstantAction(intake::in)
                )
        );

        Action cycle3 = new ParallelAction(
                drive.actionBuilder(new Pose2d(63, 15, Math.toRadians(90)))
                        .setTangent(Math.toRadians(90))
                        //                                                               robot angle             path angle
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 60), Math.toRadians(90)), Math.toRadians(90), drive.defaultVelConstraintSlow) // <---  (too hp area)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(90)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(60), drive.defaultVelConstraintSlow) // (forward to hp area again)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(90)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(60), drive.defaultVelConstraintSlow)
                        // Go back to shoot
                        .waitSeconds(.5) // wait however long it takes to intake
                        .setTangent(Math.toRadians(90))
                        //                                                               robot end angle       path end angle
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 15), Math.toRadians(90)), Math.toRadians(90)) // back to shoot

                        .build(),
                new SequentialAction(
                        // new SleepAction(2), // time till end of second spline
                        // new InstantAction(intake::out),
                        new SleepAction(0), // as need to spit out?
                        new InstantAction(intake::in)
                )
        );

        Action cycle4 = new ParallelAction(
                drive.actionBuilder(new Pose2d(63, 15, Math.toRadians(90)))
                        .setTangent(Math.toRadians(-90))
                        //                                                               robot angle             path angle
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 60), Math.toRadians(90)), Math.toRadians(90), drive.defaultVelConstraintSlow) // <---  (too hp area)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(90)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(60), drive.defaultVelConstraintSlow) // (forward to hp area again)
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 50), Math.toRadians(90)), Math.toRadians(90)) // (backup) Change these to make it *wiggle* more or less
                        .splineToLinearHeading(new Pose2d(new Vector2d(63, 59), Math.toRadians(90)), Math.toRadians(60), drive.defaultVelConstraintSlow)
                        // Go back to shoot
                        // .waitSeconds(.5) // wait however long it takes to intake
                        // .setTangent(Math.toRadians(90))
                        //                                                               robot end angle       path end angle
                        // .splineToLinearHeading(new Pose2d(new Vector2d(63, -15), Math.toRadians(270)), Math.toRadians(270)) // back to shoot

                        .build(),
                new SequentialAction(
                        //new SleepAction(2), // time till end of second spline
                        //new InstantAction(intake::out),
                        new SleepAction(0), // as need to spit out?
                        new InstantAction(intake::in)
                )
        );

        //hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
        turret.turret.setPosition(.36);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction( // should have the shooter up 24/7
                        telemetryPacket -> {
                            shooter.setVelocity(1600);
                            return true; // dont stop
                        },
                        new SequentialAction(
                                // Preloads
                                shootAction(robot),

                                // Cycle 1:
                                // Get balls for cycle 1 (spike mark)
                                new InstantAction(intake::in),
                                cycle1,
                                new InstantAction(intake::stop),
                                shootAction(robot),

                                // Cycle 2 Player area:
                                // Human
                                new InstantAction(intake::in),
                                cycle2,
                                shootAction(robot),

                                // Cycle 3/4 Player area
                                new InstantAction(intake::in),
                                cycle3,
                                shootAction(robot),

                                // Cycle 3/4 Player area
                                new InstantAction(intake::in),
                                cycle4,
                                shootAction(robot),

                                new InstantAction(()->  AutoTelopSaveState.END_AUTO_STATE = new Pose2d(0, -45, 208)),
                                new InstantAction(this::stop) // kill opmode
                        )));
        sleep(3000000); // dude are we deadass in the big 26
    }

    public Action shootAction(Hardware hardware) {
        return new SequentialAction(
                new InstantAction(()-> hardware.hood.setPosition(.22)),
                new InstantAction(()-> hardware.transfer.setPower(-.75)),
                new InstantAction(() -> hardware.gate.setPosition(Constants.TRIGGER_OPEN)),
                packet -> ( // wait till ~up to speed
                        hardware.shooter.getVelocity() < 1600 ||
                                hardware.shooter2.getVelocity() < 1600),
                new InstantAction(() -> hardware.transfer.setPower(Constants.TRANSFER_SPEED)),
                new InstantAction(()-> hardware.intake.setPower(.75)),
                new SleepAction(1.25), // reduce?
                new InstantAction(()-> hardware.intake.setPower(0)),
                new InstantAction(()-> hardware.transfer.setPower(0)),
                new InstantAction(() -> hardware.gate.setPosition(Constants.TRIGGER_CLOSE))
        );
    }
}
