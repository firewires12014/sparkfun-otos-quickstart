package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
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

@Autonomous(name = "templateAuto", group = "Templates")
public final class templateAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Non-RoadRunner hardware (intake/shooter/servos/etc)
        Hardware robot = new Hardware(hardwareMap);
        Hood hood = new Hood(hardwareMap);

        // Road Runner drive
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Example constants usage
        final int xOffset = Constants.BLUE_CLOSE_X_OFFSET;
        final int yOffset = Constants.BLUE_CLOSE_Y_OFFSET;

        // Example: pre-start mechanism configuration
        hood.down();

        // Build a small example path
        Action goToPoint = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(24 + xOffset, 0 + yOffset))
                .build();

        Action turnAndComeBack = drive.actionBuilder(new Pose2d(24 + xOffset, 0 + yOffset, 0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(0, 0))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Auto goes here

    }
}
