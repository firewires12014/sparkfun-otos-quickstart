package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous
public final class redCloseAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        int xOffset = Constants.RED_CLOSE_X_OFFSET;
        int yOffset = Constants.RED_CLOSE_Y_OFFSET;

        // Start Pose: Y flipped, Heading negated (-214 is same as 146 deg)
        Pose2d startPose = new Pose2d(-50.1, 50, Math.toRadians(125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action preload = drive.actionBuilder(new Pose2d(-50.1, 50, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-23.5, 10), Math.toRadians(125))
                .build();
        Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-20, 10, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(13, 26), Math.toRadians(80))
                .build();
        Action intakeRow2 = drive.actionBuilder(new Pose2d(13, 26, Math.toRadians(83)))
                .strafeToLinearHeading(new Vector2d(13, 72), Math.toRadians(83))
                .build();
        Action backupFromRow2 = drive.actionBuilder(new Pose2d(14, 60, Math.toRadians(83)))
                .strafeToLinearHeading(new Vector2d(13, 26), Math.toRadians(125))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(13, 26, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-23.5, 10), Math.toRadians(125))
                .build();
        Action toRow1 = drive.actionBuilder(new Pose2d(-23.5, 10, Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(-12, 25), Math.toRadians(83))
                .build();
        Action intakeRow1 = drive.actionBuilder(new Pose2d(-12, 25, Math.toRadians(83)))
                        .strafeToLinearHeading(new Vector2d(-12, 60), Math.toRadians(83))
                                .build();
        Action shoot1position = drive.actionBuilder(new Pose2d(-14, 60, Math.toRadians(83)))
                        .strafeToLinearHeading(new Vector2d( -23.5, 10), Math.toRadians(125))
                                .build();
        Action park = drive.actionBuilder(new Pose2d(-23.5, 10, Math.toRadians(125)))
                        .strafeToLinearHeading(new Vector2d(-15, 29), Math.toRadians(125))
                                .build();

        hood.setPosition(Constants.HOOD_UPPER_LIMIT);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                preload,
                shooter.shootAction(), // Shoot preload
                lineUpWithRow2,
                new InstantAction(()-> intake.in()),
                intakeRow2,
                new InstantAction(()-> intake.stop()),
                backupFromRow2,
                shootPosition,
                shooter.shootAction(), // Shoot 4-6
                toRow1,
                new InstantAction(()-> intake.in()),
                intakeRow1,
                new InstantAction(()-> intake.stop()),
                shoot1position,
                shooter.shootAction(),
                park


//
//                shootPosition,
//                shooter.shootAction() // Shoot 7-9
        ));

    }
}
