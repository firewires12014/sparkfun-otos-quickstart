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
public final class blueFarModified extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(72, -15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        Action ToStack = drive.actionBuilder(new Pose2d(72, -15, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(47, -15), Math.toRadians(270))
                .build();
        Action intakeRow = drive.actionBuilder(new Pose2d(45, -15, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(47, -70), Math.toRadians(270))
                .build();
        Action shootPosition = drive.actionBuilder(new Pose2d(47, -70, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(72, -15), Math.toRadians(180))
                .build();
        Action park = drive.actionBuilder(new Pose2d(72, -15, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(70, -30), Math.toRadians(180))
                .build();




        turret.turret.setPosition(.42);

        waitForStart();

        //hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
        //turret.turret.setPosition(Turret.middle);

        Actions.runBlocking(new SequentialAction(
                shooter.shootActionBlueFarModified(),
                park,
//                ToStack,
//                new InstantAction(()-> intake.in()),
//                intakeRow,
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

        // double check if this works
    }
}
