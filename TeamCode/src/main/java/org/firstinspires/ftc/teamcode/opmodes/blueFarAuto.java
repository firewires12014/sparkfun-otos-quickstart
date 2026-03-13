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
public final class blueFarAuto extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(63, -15, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action toRow = drive.actionBuilder(new Pose2d(63, -15, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(34, -20), Math.toRadians(270))
                .build();
        Action intakeRow = drive.actionBuilder(new Pose2d(34, -20, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(34, -70), Math.toRadians(270))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(34, -70, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(63, -15), Math.toRadians(270))
                .build();

        Action intakeHumanPlayer = drive.actionBuilder(new Pose2d(63, -15, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(63, -60), Math.toRadians(275))
                .build();
        Action backUp = drive.actionBuilder(new Pose2d(63, -60, Math.toRadians(275)))
                .strafeToSplineHeading(new Vector2d(63, -45), Math.toRadians(270))
                .build();
        Action forward = drive.actionBuilder(new Pose2d(63, -49, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(63, -59), Math.toRadians(270))
                .build();
        Action shootPosition2 = drive.actionBuilder(new Pose2d(63, -60, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(63, -15), Math.toRadians(270))
                .build();








        waitForStart();

        //hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
        turret.turret.setPosition(.63);

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



//                intakeHumanPlayer,
//                new InstantAction(()-> intake.in()),
//                backUp,
//                forward,
//                shootPosition,
//                new InstantAction(()-> intake.stop()),
//                shootPosition,
//                shooter.shootActionBlueFar(),
//                shooter.shootActionBlue(),// Shoot 4-6
//                row1LineUp,
//                new InstantAction(()-> intake.in()),
//                intakeRow1,
//                new InstantAction(()-> intake.stop()),
//                row1ShootPosition,
//                shooter.shootActionBlue(),
//                park,
                new InstantAction(()->  AutoTelopSaveState.END_AUTO_STATE = new Pose2d(0, -45, 208)


                )));
sleep(3000000);
        // double check if this works
    }
}
