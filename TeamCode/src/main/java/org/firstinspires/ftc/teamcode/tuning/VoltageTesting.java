package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
@Config
public class VoltageTesting extends LinearOpMode {
    public static double power= 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            shooter.shooter.setPower(power);
            shooter.shooter2.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("velocity", shooter.shooter.getVelocity());
            telemetry.update();


        }
        }

    }

