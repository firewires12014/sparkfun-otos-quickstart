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
        Pose2d startPose = new Pose2d(-53, 50, Math.toRadians(-214));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action preload = drive.actionBuilder(new Pose2d(-53, 48, Math.toRadians(-214)))
                .strafeToLinearHeading(new Vector2d(-20, 10), Math.toRadians(-250))
                .build();
        Action lineUpWithRow2 = drive.actionBuilder(new Pose2d(-23, 22.5, Math.toRadians(-214)))
                .strafeToLinearHeading(new Vector2d(5 + xOffset, 26 + yOffset), Math.toRadians(-256))
                .build();
        Action intakeRow2 = drive.actionBuilder(new Pose2d(5 + xOffset, 26 + yOffset, Math.toRadians(-256)))
                .strafeToLinearHeading(new Vector2d(5, 72 + yOffset), Math.toRadians(-256))
                .build();
        Action backupFromRow2 = drive.actionBuilder(new Pose2d(5, 72 + yOffset, Math.toRadians(-256)))
                .strafeToLinearHeading(new Vector2d(5 + xOffset, 26 + yOffset), Math.toRadians(-214))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(5 + xOffset, 26 + yOffset, Math.toRadians(-214)))
                .strafeToLinearHeading(new Vector2d(-23, 22.5), Math.toRadians(-214))
                .build();
        Action toGate = drive.actionBuilder(new Pose2d(-23, 22.5, Math.toRadians(-214)))
                .strafeToLinearHeading(new Vector2d(-15, 25), Math.toRadians(-256))
                .build();

        hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                preload,
                shooter.shootAction(), // Shoot preload
                lineUpWithRow2,
                intakeRow2,
                backupFromRow2,
                shootPosition,
                shooter.shootAction(), // Shoot 4-6
                toGate,

                shootPosition,
                shooter.shootAction() // Shoot 7-9
        ));

    }
}
