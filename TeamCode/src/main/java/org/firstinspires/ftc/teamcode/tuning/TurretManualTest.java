package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "Turret Manual Test", group = "Tuning")
public class TurretManualTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        telemetry.addLine("Turret Manual Test: Press A to run turret, B to reverse, X to stop");
        telemetry.addLine("Press Y to zero turret encoder (zeroForward)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                turret.setManualPower(0.5);
            } else if (gamepad1.b) {
                turret.setManualPower(-0.5);
            } else if (gamepad1.x) {
                turret.stop();
            }

            if (gamepad1.y) {
                turret.zeroForward();
            }

            telemetry.addData("EncoderTics", turret.turretEncoder.getCurrentPosition());
            telemetry.addData("TurretPosDeg", turret.getRobotRelativeDeg());
            telemetry.addData("Debug", turret.getDebugString());
            telemetry.update();
        }
    }
}
