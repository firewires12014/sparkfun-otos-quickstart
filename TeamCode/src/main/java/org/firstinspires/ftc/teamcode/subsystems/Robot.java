package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public class Robot {
    public Intake intake;
    public Hang hang;
    public Lift lift;
    public Outtake outtake;
    public SparkFunOTOSDrive drive;

    public Robot(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        hang = new Hang(hardwareMap);
        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,0));

    }

    public void update () {
        intake.update();
        hang.update();
        lift.update();
        outtake.update();
        drive.updatePoseEstimate();

    }
}
