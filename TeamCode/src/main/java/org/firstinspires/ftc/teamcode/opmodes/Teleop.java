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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Teleop", group = "Linear OpMode")
public class Teleop extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Subsystems
        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Ensure subsystems are in starting state
        transfer.triggerClose();

        waitForStart();
        runtime.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // =========================================================================
            // GAMEPAD 1 CONTROLS
            // =========================================================================

            // --- DRIVE CONTROL ---
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);

            // =========================================================================
            // GAMEPAD 2 CONTROLS
            // =========================================================================

            // --- SHOOTER & INTAKE CONTROL ---

            // Shooter Trigger Logic
            double triggerVal = gamepad2.left_trigger;
            boolean isShooting = triggerVal > 0.001;

            if (gamepad2.cross && triggerVal == 0) {
                // Intake Logic: Intake Only
                intake.in();
                transfer.run();
                shooter.update(isShooting);
            } else if (gamepad2.circle && triggerVal == 0) {
                // Reverse Logic: Outtake and Reverse Systems
                intake.out();
                transfer.reverse();
                shooter.reverse();
            } else if (gamepad2.right_trigger > 0 && triggerVal == 0) {
                // Intake Only (Right Trigger)
                intake.in();
                transfer.stop();
                transfer.triggerClose();
                telemetry.addLine("Trigger closed");
                shooter.update(isShooting);
            } else if (gamepad2.right_trigger == 0 && triggerVal == 0) {
                // Idle State
                intake.stop();
                transfer.stop();
                shooter.update(isShooting);
            } else {
                // Default: Update shooter state
                shooter.update(isShooting);
            }

            // Transfer Logic: Run transfer when shooting
            if (triggerVal > 0.001) {
                transfer.run();
                intake.in();
            } else if (!gamepad2.cross && !gamepad2.circle && gamepad2.right_trigger == 0) {
                intake.stop();
            }

            // --- TURRET CONTROL (Bumpers) ---
            if (gamepad2.left_bumper) {
                turret.rotateLeft();
            } else if (gamepad2.right_bumper) {
                turret.rotateRight();
            } else {
                turret.stop();
            }

            // --- HOOD CONTROL (D-Pad) ---
            if (gamepad2.dpad_right) {
                hood.up(); // Preset Up
            } else if (gamepad2.dpad_left) {
                hood.down(); // Preset Down
            } else if (gamepad2.dpad_up) {
                hood.moveHood(0.005); // Fine Adjustment Up
            } else if (gamepad2.dpad_down) {
                hood.moveHood(-0.005); // Fine Adjustment Down
            }

            drive.update();

            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Velocity", drive.shooter.getVelocity());
            telemetry.addData("Target Velo", (triggerVal > 0.001) ? Constants.SHOOTER_VELOCITY : 0.0);
            telemetry.addData("Hood Pos", hood.getPosition());
            telemetry.update();
        }
    }
}
