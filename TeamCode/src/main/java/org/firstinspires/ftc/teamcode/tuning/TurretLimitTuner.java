package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Turret Limit Tuner", group = "Calibration")
public class TurretLimitTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap, telemetry);

        telemetry.addLine("Turret Limit Tuner");
        telemetry.addLine("- Point turret straight forward, then press (A) to zero.");
        telemetry.addLine("- Use left stick X to move turret.");
        telemetry.addLine("- Move to LEFT hard stop and press (X) to save left limit.");
        telemetry.addLine("- Move to RIGHT hard stop and press (B) to save right limit.");
        telemetry.addLine("- Limits are saved as degrees relative to forward.");
        telemetry.update();

        waitForStart();

        double savedLeftDeg = Turret.LIMIT_LEFT_DEG;
        double savedRightDeg = Turret.LIMIT_RIGHT_DEG;

        while (opModeIsActive()) {
            // Manual jog
            double cmd = -gamepad1.left_stick_x; // stick right => negative => rotate right
            turret.setManualPower(cmd);

            // Zero forward
            if (gamepad1.a) {
                turret.zeroForward();
            }

            // Save left limit (+deg)
            if (gamepad1.x) {
                savedLeftDeg = Math.max(0.0, turret.getRobotRelativeDeg());
                turret.setLimitsDeg(savedLeftDeg, savedRightDeg);
            }

            // Save right limit (-deg)
            if (gamepad1.b) {
                savedRightDeg = Math.max(0.0, -turret.getRobotRelativeDeg());
                turret.setLimitsDeg(savedLeftDeg, savedRightDeg);
            }

            // Output
            telemetry.addData("Turret/deg", turret.getRobotRelativeDeg());
            telemetry.addData("Turret/encoderTicks", turret.turretEncoder.getCurrentPosition());
            telemetry.addData("Turret/zeroTicks", turret.getZeroTicks());
            telemetry.addData("Tuned/leftLimitDeg", savedLeftDeg);
            telemetry.addData("Tuned/rightLimitDeg", savedRightDeg);
            telemetry.addLine("Copy these into Turret.LIMIT_LEFT_DEG and Turret.LIMIT_RIGHT_DEG");
            telemetry.update();

            // keep update signature satisfied (pose not needed here)
            turret.update(new Pose2d(0, 0, 0));
        }

        turret.stop();
    }
}
