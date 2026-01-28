package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Turret Limit Tuner", group = "Tuning")
public class TurretLimitTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(hardwareMap);

        // Ensure turret servo is stopped so we can move it by hand
        robot.turret.setPower(0);

        // If the encoder is connected to a motor port that supports it, we might want
        // to reset it
        // robot.turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Turret Limit Tuner");
        telemetry.addLine("Move turret by hand to read values");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int currentPos = robot.turretEncoder.getCurrentPosition();
            double currentRad = currentPos * Constants.TICKS_TO_RADIANS;
            double currentDeg = Math.toDegrees(currentRad);

            telemetry.addData("Encoder Ticks", currentPos);
            telemetry.addData("Degrees", currentDeg);
            telemetry.addData("Radians", currentRad);
            telemetry.update();
        }
    }
}