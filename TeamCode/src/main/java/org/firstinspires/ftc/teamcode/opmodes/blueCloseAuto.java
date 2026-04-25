package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.teamcode.opmodes.AutoTelopSaveState;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
public final class blueCloseAuto extends LinearOpMode {

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

                Pose2d startPose = new Pose2d(-53, -50, Math.toRadians(214));
                MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

                Action preload = drive.actionBuilder(new Pose2d(-53, -50, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-19, -26), Math.toRadians(214))
                                .build();
                Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-19, -26, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(4 + xOffset, -26 + yOffset), Math.toRadians(253))
                                .build();
                Action intakeRow2 = drive.actionBuilder(new Pose2d(4 + xOffset, -26 + yOffset, Math.toRadians(253)))
                                .strafeToLinearHeading(new Vector2d(4, -74 + yOffset), Math.toRadians(253))
                                .build();
                Action backupFromRow2 = drive.actionBuilder(new Pose2d(3, -72 + yOffset, Math.toRadians(253)))
                                .strafeToLinearHeading(new Vector2d(5 + xOffset, -26 + yOffset), Math.toRadians(214))
                                .build();
                Action row2ShootPosition = drive.actionBuilder(new Pose2d(5 + xOffset, -26 + yOffset, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-19, -26), Math.toRadians(214))
                                .build();
                Action row1LineUp = drive.actionBuilder(new Pose2d(-19, -26, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-1.5, -20), Math.toRadians(253))
                                .build();
                Action intakeRow1 = drive.actionBuilder(new Pose2d(-1.5, -20, Math.toRadians(253)))
                        .strafeToLinearHeading(new Vector2d(-17, -73), Math.toRadians(253))
                        .build();
                Action row1ShootPosition = drive.actionBuilder(new Pose2d(-17, -73, Math.toRadians(253)))
                        .strafeToLinearHeading(new Vector2d(-19, -26), Math.toRadians(214))
                        .build();
                Action park = drive.actionBuilder(new Pose2d(-19, -26, Math.toRadians(214)))
                        .strafeToLinearHeading(new Vector2d(0, -45), Math.toRadians(208))
                        .build();

                Action toGate = drive.actionBuilder(new Pose2d(-19, -25, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-1 + xOffset, -40 + yOffset), Math.toRadians(256))
                                .build();
                Action openGate = drive.actionBuilder(new Pose2d(-1 +xOffset, -40 + yOffset, Math.toRadians(256)))
                                .strafeToLinearHeading(new Vector2d(4, -80), Math.toRadians(220))
                                .build();



                waitForStart();

                hood.setPosition(0);
                turret.turret.setPosition(Turret.middle);

                Actions.runBlocking(new ParallelAction(
                        new SequentialAction(
                                preload,
                                shooter.shootActionBlue(), // Shoot preload
                               // new InstantAction(()-> shooter.stop()),
                                lineUpWithRow2,
                                new InstantAction(()-> intake.in()),
                                intakeRow2,
                                new SleepAction(.5),
                                backupFromRow2,
                                new InstantAction(()-> intake.stop()),
                                row2ShootPosition,
                                shooter.shootActionBlue(),// Shoot 4-6
                               // new InstantAction(()-> shooter.stop()),
                                row1LineUp,
                                new InstantAction(()-> intake.in()),
                                intakeRow1,
                                new SleepAction(.5),
                                row1ShootPosition,
                                new InstantAction(()-> intake.stop()),
                                shooter.shootActionBlue(),
                               // new InstantAction(()-> shooter.stop()),
                                park
                        ),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket packet) {
                                shooter.setVelocity(1200);
                                return true;
                            }
                        }

                        ));
            new InstantAction(()->  AutoTelopSaveState.END_AUTO_STATE = new Pose2d(0, -45, 208));


            // double check if this works
        }
}
