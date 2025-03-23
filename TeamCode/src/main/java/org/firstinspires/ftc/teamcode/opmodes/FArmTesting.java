package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.FArm;

@Config
@TeleOp
//@Disabled
public class FArmTesting extends LinearOpMode {
    public static double leftPosition = 0.5;
    public static double rightPosition = 0.54;
    public static double wristPosition = 0.5;
    public static double clawPosition = 0.5;

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Class(s)
        FArm farm = new FArm(hardwareMap);
        Servo pivot = hardwareMap.get(Servo.class, "intakePivot");

        // Any pre start init shi


        waitForStart();
        prevLoop = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
            // Insert actual code
            pivot.setPosition(0.92);

            farm.left.setPosition(leftPosition);
            farm.right.setPosition(rightPosition);

            farm.wrist.setPosition(wristPosition);

            farm.grab.setPosition(clawPosition);

            farm.updatePID();

            farm.update();

            farm.manualControl(-gamepad2.right_stick_y);

            telemetry.addLine("Telemetry Data"); // Add telemetry below this
            telemetry.addData("Target Position", FArm.targetPosition);
            telemetry.addData("Lift Position", farm.lift.getCurrentPosition());
            telemetry.addData("Lift1 Current", farm.lift.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Lift2 Current", farm.lift.getCurrent(CurrentUnit.MILLIAMPS));
            loopTimeMeasurement(telemetry); // Don't update telemetry again, this method already does that
        }
    }


    public void loopTimeMeasurement(Telemetry telemetry) {
        double currTime = System.nanoTime() / 1e9;
        double delta = currTime - prevLoop;
        telemetry.addData("loop time", delta / 1000);
        telemetry.addData("hz", 1 / delta);
        telemetry.update();
        prevLoop = System.nanoTime() / 1e9;
    }
}
