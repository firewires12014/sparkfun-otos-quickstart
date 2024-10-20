//import com.acmerobotics.roadrunner.Math;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.messages.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.subsystems.intake;

import java.util.Random;

@Autonomous
public class auto extends LinearOpMode{
    Pose2d beginPose = new Pose2d(0, 0, 0);
    public static Pose2d firstPosition = new Pose2d(0, 40, Math.toRadians(-90));
    public static Pose2d secondPosition = new Pose2d(0, 0, Math.toRadians(0));

    public void update (){

    }

        @Override
        public void runOpMode() throws InterruptedException {

            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
            intake intake = new intake(hardwareMap);
            AutoActionScheduler scheduler = new AutoActionScheduler(this::update);

            scheduler.addAction(drive.actionBuilder(beginPose)
                    .afterDisp(0, new SequentialAction(intake.intakeOff()))
                   // .splineToSplineHeading(firstPosition, firstPosition.heading.toDouble())
                    .afterDisp(0, new SequentialAction( intake.intakeOn() ))
                    .turnTo(firstPosition.heading)
                    .waitSeconds(1)
                   // .splineToConstantHeading(secondPosition.position, secondPosition.heading.toDouble())
                    .build()
            );
            //scheduler.addAction(intake.intakeOn());

            waitForStart();

            scheduler.run();
            sleep(400000000);
        }
    }
