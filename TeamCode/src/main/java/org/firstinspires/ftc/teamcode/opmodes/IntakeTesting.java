package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Config
@TeleOp
//@Disabled
public class IntakeTesting extends LinearOpMode {
    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        Intake intake = new Intake(hardwareMap);

        // Any pre start init shi


        waitForStart();
        prevLoop = System.nanoTime() / 1e6;
        while (opModeIsActive() && !isStopRequested()) {
            // Insert actual code
            intake.updatePID();
            intake.update();

            telemetry.addLine("Intake Data"); // Add telemetry below this
            telemetry.addData("Target Position", Intake.targetPosition);
            telemetry.addData("Encoder Position", intake.extension.getCurrentPosition());
            telemetry.addData("Extension Power", intake.extension.getPower());
            telemetry.addData("Spin Power", intake.spin.getPower());
            telemetry.addData("Intake Current", intake.extension.getCurrent(CurrentUnit.MILLIAMPS));
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }


    public void loopTimeMeasurement(Telemetry telemetry) {
        double currTime = System.nanoTime() / 1e6;
        double delta = currTime - prevLoop;
        telemetry.addData("loop time", delta);
        telemetry.addData("hz", 1 / delta);
        telemetry.update();
        prevLoop = System.nanoTime() / 1e6;
    }
}
