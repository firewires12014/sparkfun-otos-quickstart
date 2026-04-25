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

import org.firstinspires.ftc.teamcode.opmodes.AutoTelopSaveState;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
public final class redFarAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(-53, 15, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        Action toRow = drive.actionBuilder(new Pose2d(-63, 15, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(34, 20), Math.toRadians(90))
                .build();
        Action intakeRow = drive.actionBuilder(new Pose2d(34, 20, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(34, 70), Math.toRadians(90))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(34, 70, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(63, 15), Math.toRadians(90))
                .build();

        Action intakeHumanPlayer = drive.actionBuilder(new Pose2d(63, 15, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(63, 60), Math.toRadians(85))
                .build();
        Action backUp = drive.actionBuilder(new Pose2d(63, 60, Math.toRadians(85)))
                .strafeToSplineHeading(new Vector2d(63, 45), Math.toRadians(90))
                .build();
        Action forward = drive.actionBuilder(new Pose2d(63, 45, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(63, 59), Math.toRadians(90))
                .build();
        Action shootPosition2 = drive.actionBuilder(new Pose2d(63, 59, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(63, 15), Math.toRadians(90))
                .build();
        Action park = drive.actionBuilder(new Pose2d(63, 15, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(60, 35), Math.toRadians(90))
                .build();









        waitForStart();

        //hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
        turret.turret.setPosition(.4);

        Actions.runBlocking(new SequentialAction(
                shooter.shootActionBlueFar(),
                toRow,
                new InstantAction(()-> intake.in()),
                new InstantAction(()-> shooter.stop()),
                intakeRow,
                shootPosition,
                new InstantAction(()-> intake.stop()),
                shooter.shootActionBlueFar(),
                new InstantAction(()-> intake.in()),
                new InstantAction(()-> shooter.stop()),
                intakeHumanPlayer,
                backUp,
                new InstantAction(()-> intake.out()),
                //new SleepAction(.),
                new InstantAction(()-> intake.in()),
                forward,
                shootPosition2,
                shooter.shootActionBlueFar(),
                park,

                new InstantAction(()->  AutoTelopSaveState.END_AUTO_STATE = new Pose2d(0, -45, 208)


                )));

        // double check if this works
    }
}
