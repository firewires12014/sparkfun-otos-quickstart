import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.messages.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.subsystems.intake;
@Autonomous
public class auto extends LinearOpMode{
    Pose2d beginPose = new Pose2d(0, 0, 0);

    public void update (){

    }

        @Override
        public void runOpMode() throws InterruptedException {


            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
            intake intake = new intake(hardwareMap);
            AutoActionScheduler scheduler = new AutoActionScheduler(this::update);

            scheduler.addAction(drive.actionBuilder(beginPose)
                    .afterDisp(15, intake.intakeOff())
                            .strafeTo(new Vector2d (0, 40))

                    .build()
            );
            scheduler.addAction(intake.intakeOn());

                waitForStart();

                scheduler.run();

        }
    }
