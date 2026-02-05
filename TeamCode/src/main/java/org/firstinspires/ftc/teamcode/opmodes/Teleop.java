package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name = "Teleop", group = "Linear OpMode")
public class Teleop extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    public static double targetX = -67;
    public static double targetY = 67;
    public static boolean isBlue = true;
    public static double hoodPosition = 0;


    public static boolean autoTurret = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Subsystems
        Drive drive = new Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
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

            drive.update();
            Pose2d pose = drive.getPose();

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
            double leftTriggerVal = gamepad2.left_trigger;
            double rightTriggerVal = gamepad2.right_trigger;
            boolean isShooting = leftTriggerVal > 0.001;

            if (gamepad2.cross && leftTriggerVal == 0) {
                // Intake Logic: Intake Only
                intake.in();
                transfer.run();
                shooter.update(isShooting);
            } else if (gamepad2.circle && leftTriggerVal == 0) {
                // Reverse Logic: Outtake and Reverse Systems
                intake.out();
                transfer.reverse();
                shooter.reverse();
            } else if (rightTriggerVal > 0 && leftTriggerVal == 0) {
                // Intake Only (Right Trigger)
                intake.in();
                transfer.stop();
                transfer.triggerClose();
                telemetry.addLine("Trigger closed");
                shooter.update(isShooting);
            } else if (rightTriggerVal == 0 && leftTriggerVal == 0) {
                // Idle State
                intake.stop();
                transfer.stop();
                shooter.update(isShooting);
            } else {
                // Default: Update shooter state
                shooter.update(isShooting);
            }

            // Transfer Logic: Run transfer when shooting
            if (leftTriggerVal > 0.001) {
//                transfer.run();
//                intake.in();
            } else if (!gamepad2.cross && !gamepad2.circle && rightTriggerVal == 0) {
                intake.stop();
            }

            if (gamepad2.triangle) autoTurret = false;
            if (gamepad2.square) autoTurret = true;

            // --- TURRET CONTROL ---
            double targetAnlge = findTargetAngle(new Pose2d(targetX, targetY, 0), pose);
            if (!autoTurret) {
                double stick = gamepad2.left_stick_x;
                double deadzone = 0.05;
                if (stick < -deadzone) {
                    turret.increment(gamepad2.left_stick_x);
                } else if (stick > deadzone) {
                    turret.increment(gamepad2.left_stick_x);
                }

            } else {
                double adjustment = 0;
                if (isBlue)
                    adjustment = 0;
                else adjustment = 0;
                turret.setAngle(-targetAnlge + adjustment);
            }

            // --- HOOD CONTROL (D-Pad) ---
            double distance = Math.sqrt(Math.pow(targetX - pose.position.x, 2)+Math.pow(targetY - pose.position.y, 2));
            if (!autoTurret) {
                if (gamepad2.dpad_up) {
                    hood.setPosition(Constants.HOOD_UPPER_LIMIT);
                }

                if (gamepad2.dpad_right) {
                    hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
                }

                if (gamepad2.dpad_down) {
                    hood.setPosition(Constants.HOOD_LOWER_LIMIT);
                }

            }
            else {

                hood.setPosition(hood.lerp(distance));
            }

            if (gamepad2.left_bumper) {
                lift.up();
            }

            if (gamepad2.right_bumper) {
                lift.down();
            }

            if (gamepad1.triangle) {
                if (isBlue) {
                    gamepad1.setLedColor(0, 0, 255, -1);
                    drive.setPose(new Pose2d(0, 63, Math.toRadians(90)));
                    targetX = -67;
                }
                else {
                    gamepad1.setLedColor(255, 0, 0, -1);
                    drive.setPose(new Pose2d(0, 63, Math.toRadians(90)));
                    targetX = 67;
                }

            }

            if (gamepad1.circle) {
                gamepad1.setLedColor(255, 0, 0, -1);
                isBlue = false;
            }

            if (gamepad1.cross) {
                gamepad1.setLedColor(0, 0, 255, -1);
                isBlue = true;
            }



            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Robot Pose", "Pose: "+pose.position + "\tHeading: "+Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("targetAngle", targetAnlge);
            telemetry.addData("Velocity", drive.shooter.getVelocity());
            telemetry.addData("Target Velo", (leftTriggerVal > 0.001) ? Constants.SHOOTER_VELOCITY : 0.0);
            telemetry.addData("Hood Pos", hood.getPosition());
            telemetry.addData("isBlue", isBlue);
            telemetry.addData("hoodDistance", distance);
            telemetry.update();
        }
    }
    public double findTargetAngle (Pose2d target, Pose2d current) {
        Pose2d delta = Pose2d.exp(target.minus(current));
        return Math.toDegrees(Math.atan2(delta.position.y, delta.position.x));
    }
}
