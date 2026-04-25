package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
public final class redCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        int xOffset = Constants.RED_CLOSE_X_OFFSET;
        int yOffset = Constants.RED_CLOSE_Y_OFFSET;

        // Start Pose: Y flipped, Heading negated (-214 is same as 146 deg)
        Pose2d startPose = new Pose2d(-50.1, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action preload = drive.actionBuilder(new Pose2d(-50.1, 50, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-19, 13), Math.toRadians(130))
                .build();
        Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-21, 10, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(13, 26), Math.toRadians(88))
                .build();
        Action intakeRow2 = drive.actionBuilder(new Pose2d(13, 26, Math.toRadians(88)))
                .strafeToLinearHeading(new Vector2d(13, 72), Math.toRadians(88))
                .build();
        Action backupFromRow2 = drive.actionBuilder(new Pose2d(13, 72, Math.toRadians(88)))
                .strafeToLinearHeading(new Vector2d(15, 26), Math.toRadians(130))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(15, 26, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-21, 15), Math.toRadians(130))
                .build();
        Action toRow1 = drive.actionBuilder(new Pose2d(-23, 12, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-10, 25), Math.toRadians(86))
                .build();
        Action intakeRow1 = drive.actionBuilder(new Pose2d(-10, 25, Math.toRadians(86)))
                        .strafeToLinearHeading(new Vector2d(-10, 60), Math.toRadians(86))
                                .build();
        Action shoot1position = drive.actionBuilder(new Pose2d(-10 , 60, Math.toRadians(86)))
                        .strafeToLinearHeading(new Vector2d( -20, 14), Math.toRadians(130))
                                .build();
        Action park = drive.actionBuilder(new Pose2d(-23, 12, Math.toRadians(130)))
                        .strafeToLinearHeading(new Vector2d(-4, 32), Math.toRadians(125))
                                .build();

        hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
        turret.turret.setPosition(.495);


        waitForStart();

        Actions.runBlocking(new ParallelAction(

                new SequentialAction(
                preload,
                shooter.shootActionRed(), // Shoot preload
                lineUpWithRow2,
                new InstantAction(()-> intake.in()),
                intakeRow2,
                backupFromRow2,
                new InstantAction(()-> intake.stop()),

                new ParallelAction(
                        shootPosition,
                        new SequentialAction(
                                new InstantAction(()-> intake.out()),
                                new SleepAction(.05),
                                new InstantAction(()-> intake.stop())
                                )

                ),
                shooter.shootActionRed(), // Shoot 4-6
                toRow1,
                new InstantAction(()-> intake.in()),
                intakeRow1,
                        new ParallelAction(
                                shoot1position,
                                new SequentialAction(
                                        new InstantAction(()-> intake.out()),
                                        new SleepAction(.05),
                                        new InstantAction(()-> intake.stop())
                                )

                        ),
                new InstantAction(()-> intake.stop()),
                shooter.shootActionRed(),
                park
        ),
        new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter.setVelocity(1175);
                return true;
            }
        }
        ));
        AutoTelopSaveState.END_AUTO_STATE = new Pose2d(29, -15, 125); // double check if this works
    }
}
