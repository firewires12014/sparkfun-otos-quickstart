package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public final class blueCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int xOffset = Constants.BLUE_CLOSE_X_OFFSET;
        int yOffset = Constants.BLUE_CLOSE_Y_OFFSET;

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
        Action shoot = drive.actionBuilder(new Pose2d(5 + xOffset, -26 + yOffset, Math.toRadians(214)))
                .strafeToLinearHeading(new Vector2d(-23, -22.5), Math.toRadians(214))
                .build();
        Action toGate = drive.actionBuilder(new Pose2d(-23, -22.5, Math.toRadians(214)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(8, -30, Math.toRadians(-124)), Math.toRadians(-53))
                .build();
        Action openGate = drive.actionBuilder(new Pose2d(8, -30, Math.toRadians(-124)))
                .setTangent(-53)
                .splineToLinearHeading(new Pose2d(8, -72, Math.toRadians(-124)), Math.toRadians(-53))
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                preload,
                lineUpWithRow2,
                intakeRow2,
                backupFromRow2,
                shoot,
                toGate,
                openGate
        ));
    }
}
