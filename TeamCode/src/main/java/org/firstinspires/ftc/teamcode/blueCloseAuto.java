package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous

public final class blueCloseAuto extends LinearOpMode {
    public static int xOffset = 12;
    public static int yOffset = -14;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-53, -50, Math.toRadians(214));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action preload = drive.actionBuilder(new Pose2d(-53, -48, Math.toRadians(214)))
                .strafeToLinearHeading(new Vector2d(-23, -22.5), Math.toRadians(214))
                .build();
        Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-23, -22.5, Math.toRadians(214)))
                        .strafeToLinearHeading(new Vector2d(5 + xOffset, -26 + yOffset), Math.toRadians(256))
                        .build();
        Action intakeRow2 = drive.actionBuilder(new Pose2d(5 + xOffset, -26 + yOffset, Math.toRadians(256)))
                        .strafeToLinearHeading(new Vector2d(5, -72 + yOffset), Math.toRadians(256))
                        .build();
        Action backupFromRow2 = drive.actionBuilder(new Pose2d(5, -72 + yOffset, Math.toRadians(256)))
                        .strafeToLinearHeading(new Vector2d(5 + xOffset, -26 + yOffset), Math.toRadians(214))
                        .build();
        Action shoot = drive.actionBuilder(new Pose2d(5+xOffset, -26+yOffset, Math.toRadians(214)))
                        .strafeToLinearHeading(new Vector2d(-23, -22.5), Math.toRadians(214))
                        .build();
        Action toGate = drive.actionBuilder(new Pose2d(-23, -22.5, Math.toRadians(214)))
                        .strafeToLinearHeading(new Vector2d(0, -60 + yOffset), Math.toRadians(256))
                        .build();


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                preload,
                lineUpWithRow2,
                intakeRow2,
                backupFromRow2,
                shoot,
                toGate
        ));
    }
}

