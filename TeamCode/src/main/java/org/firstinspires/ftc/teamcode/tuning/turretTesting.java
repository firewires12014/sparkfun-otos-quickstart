package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
@Config
public class turretTesting extends LinearOpMode {
    public static double turretPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Turret turret = new Turret(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Changing to: ", turretPosition);
            telemetry.update();
            turret.turret.setPosition(turretPosition);
        }

    }
}
