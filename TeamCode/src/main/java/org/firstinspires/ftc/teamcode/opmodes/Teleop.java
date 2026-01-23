package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Teleop", group = "Linear OpMode")
public class Teleop extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Hood hood = new Hood(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        transfer.triggerClose();

        waitForStart();
        runtime.reset();

        double targetVel = 0.0;
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);

            // Smooth ramp for shooter velocity:
            double triggerVal = gamepad2.left_trigger;
            if (triggerVal > 0.001) {
                targetVel = Constants.SHOOTER_VELOCITY;
                transfer.triggerOpen();
                if (drive.shooter.getVelocity() >= Constants.SHOOTER_VELOCITY) {
                    intake.in();
                    transfer.run();
                }
            } else {
                transfer.triggerClose();
                targetVel = 0;
            }

            drive.shooter.setVelocity(targetVel);

//            if (gamepad1.circle) {
//                transfer.triggerClose();
//                telemetry.addLine("Trigger closed");
//            }

            if (gamepad2.cross && triggerVal == 0) {
                intake.in();
                transfer.run();
            } else if (gamepad2.circle && triggerVal == 0) {
                intake.out();
                transfer.reverse();
                drive.shooter.setPower(-.5);
            } else if (gamepad2.right_trigger > 0  && triggerVal == 0) {
                intake.in();
                transfer.stop();
                transfer.triggerClose();
                telemetry.addLine("Trigger closed");
            }  else if (gamepad2.right_trigger == 0  && triggerVal == 0) {
                intake.stop();
                transfer.stop();
            }
//            else  {
//                intake.stop();
//                transfer.stop();
//            }

            // Transfer runs only when shooting
            if (triggerVal > 0.001) {
                transfer.run();
            }

            if (gamepad2.left_bumper) {
                turret.rotateLeft();
            } else if (gamepad2.right_bumper) {
                turret.rotateRight();
            } else {
                turret.stop();
            }

            if (gamepad2.dpad_up) {
                hood.up();
            }

            if (gamepad2.dpad_down) {
                hood.down();
            }



            drive.update();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Velocity", drive.shooter.getVelocity());
            telemetry.addData("Target Velo", targetVel);
            telemetry.update();
        }
    }
}

