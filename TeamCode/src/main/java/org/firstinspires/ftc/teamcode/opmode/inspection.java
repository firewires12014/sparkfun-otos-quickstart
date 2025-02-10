package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Inspection", group="Into the Deep")

public class inspection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}




