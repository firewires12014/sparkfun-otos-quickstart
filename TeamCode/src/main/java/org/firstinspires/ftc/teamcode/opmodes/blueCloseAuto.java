package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Autonomous
public final class blueCloseAuto extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
                Hardware robot = new Hardware(hardwareMap);
                Hood hood = new Hood(hardwareMap);
                Shooter shooter = new Shooter(hardwareMap);
                Intake intake = new Intake(hardwareMap);
                Transfer transfer = new Transfer(hardwareMap);

                int xOffset = Constants.BLUE_CLOSE_X_OFFSET;
                int yOffset = Constants.BLUE_CLOSE_Y_OFFSET;

                Pose2d startPose = new Pose2d(-53, -50, Math.toRadians(214));
                MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

                Action preload = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(205)))
                                .strafeToLinearHeading(new Vector2d(-19, -27), Math.toRadians(214))
                                .build();
                Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-23, -22.5, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(3 + xOffset, -26 + yOffset), Math.toRadians(256))
                                .build();
                Action intakeRow2 = drive.actionBuilder(new Pose2d(3 + xOffset, -26 + yOffset, Math.toRadians(256)))
                                .strafeToLinearHeading(new Vector2d(3, -72 + yOffset), Math.toRadians(256))
                                .build();
                Action backupFromRow2 = drive.actionBuilder(new Pose2d(3, -72 + yOffset, Math.toRadians(256)))
                                .strafeToLinearHeading(new Vector2d(5 + xOffset, -26 + yOffset), Math.toRadians(214))
                                .build();
                Action row2ShootPosition = drive.actionBuilder(new Pose2d(5 + xOffset, -26 + yOffset, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-13, -26), Math.toRadians(208))
                                .build();
                Action row2LineUp = drive.actionBuilder(new Pose2d(-13, -26, Math.toRadians(208)))
                                .strafeToLinearHeading(new Vector2d(-3, -26), Math.toRadians(256))
                                .build();
                Action intakeRow1 = drive.actionBuilder(new Pose2d(-3, -26, Math.toRadians(256)))
                        .strafeToLinearHeading(new Vector2d(-15, -67), Math.toRadians(256))
                        .build();
                Action row1ShootPosition = drive.actionBuilder(new Pose2d(-15, -67, Math.toRadians(256)))
                        .strafeToLinearHeading(new Vector2d(-10, -29), Math.toRadians(208))
                        .build();
                Action park = drive.actionBuilder(new Pose2d(-10, -29, Math.toRadians(208)))
                        .strafeToLinearHeading(new Vector2d(0, -45), Math.toRadians(208))
                        .build();

                Action toGate = drive.actionBuilder(new Pose2d(-19, -25, Math.toRadians(214)))
                                .strafeToLinearHeading(new Vector2d(-1 + xOffset, -40 + yOffset), Math.toRadians(256))
                                .build();
                Action openGate = drive.actionBuilder(new Pose2d(-1 +xOffset, -40 + yOffset, Math.toRadians(256)))
                                .strafeToLinearHeading(new Vector2d(4, -80), Math.toRadians(220))
                                .build();



                waitForStart();

                hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);

                Actions.runBlocking(new SequentialAction(
                                preload,
                                shooter.shootAction(), // Shoot preload
                                lineUpWithRow2,
                                new InstantAction(()-> intake.in()),
                                intakeRow2,
                                new InstantAction(()-> intake.stop()),
                                backupFromRow2,
                                row2ShootPosition,
                                shooter.shootAction(),// Shoot 4-6
                                row2LineUp,
                                new InstantAction(()-> intake.in()),
                                intakeRow1,
                                new InstantAction(()-> intake.stop()),
                                row1ShootPosition,
                                shooter.shootAction(),
                                park


//                                shooter.shootAction() // Shoot 7-9

                ));


        }
}
