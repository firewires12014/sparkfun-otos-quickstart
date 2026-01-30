package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
        Lift lift = new Lift(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        // Pinpoint localizer provides a real heading/position from the GoBilda Pinpoint
        // device. Use it directly for turret targeting instead of drive.getPose()
        PinpointLocalizer pinpoint = null;
        try {
            pinpoint = new PinpointLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0, 0, 0));
        } catch (Exception e) {
            telemetry.addData("PinpointInit", "failed: " + e.getMessage());
            telemetry.update();
        }

        // Also try to get the Rev Hub IMU as a fallback for heading
        IMU imu = null;
        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) {
            telemetry.addData("IMUInit", "failed: " + e.getMessage());
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Ensure subsystems are in starting state
        transfer.triggerClose();

        waitForStart();
        runtime.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        // Track previous triangle state to detect rising edge (press)
        boolean prevTriangle = false;
        // Track previous robot heading to compute angular velocity (rad/sec)
        Pose2d startingPose = drive.getPose();
        double prevHeading = (startingPose != null) ? startingPose.heading.toDouble() : 0.0;

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Update pinpoint localizer (if available) so we have a fresh heading
            Pose2d pinpointPose = null;
            GoBildaPinpointDriver.DeviceStatus pinStatus = null;
            if (pinpoint != null) {
                pinpoint.update();
                pinpointPose = pinpoint.getPose();
                pinStatus = pinpoint.getDeviceStatus();
            }

            // Always show Pinpoint / IMU status and raw heading to help debugging
            if (pinpoint != null) {
                telemetry.addData("PinStatus", String.valueOf(pinStatus));
                try {
                    telemetry.addData("PinRawDeg", Math.toDegrees(pinpoint.getRawHeadingRadians()));
                } catch (Exception ignored) {
                    telemetry.addData("PinRawDeg", "err");
                }
            } else {
                telemetry.addData("PinStatus", "n/a");
                telemetry.addData("PinRawDeg", "n/a");
            }
            if (imu != null) {
                try {
                    telemetry.addData("IMUHeadingDeg", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
                } catch (Exception ignored) {
                    telemetry.addData("IMUHeadingDeg", "err");
                }
            } else {
                telemetry.addData("IMUHeadingDeg", "n/a");
            }

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

            // --- TURRET CONTROL ---
            if (gamepad2.triangle) {
                telemetry.addLine("Targeting");
                // Prefer Pinpoint when it's READY; otherwise fall back to the Rev Hub IMU.
                Pose2d usePose = null;
                boolean pinReady = (pinStatus != null && pinStatus == GoBildaPinpointDriver.DeviceStatus.READY);
                double pinRawHeading = Double.NaN;
                if (pinpoint != null) {
                    try {
                        pinRawHeading = pinpoint.getRawHeadingRadians();
                    } catch (Exception ignored) {
                    }
                }

                if (pinReady && pinpointPose != null) {
                    usePose = pinpointPose;
                } else if (!Double.isNaN(pinRawHeading) && Math.abs(pinRawHeading) > 1e-6) {
                    // Pinpoint isn't fully READY for pose but reports a non-zero raw heading; use that
                    usePose = new Pose2d(0.0, 0.0, pinRawHeading);
                } else if (imu != null) {
                    // Read IMU yaw as the heading
                    YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                    double imuHeading = ypr.getYaw(AngleUnit.RADIANS);
                    usePose = new Pose2d(0.0, 0.0, imuHeading);
                } else {
                    usePose = drive.getPose();
                }
                // On the rising edge of triangle, capture the current robot heading as the
                // world-locked turret target so the turret will move to robot-forward (0
                // robot-relative) and then compensate for robot rotation.
                if (!prevTriangle) {
                    turret.captureWorldHeadingAsRobotForward(usePose);
                }
                // Use the turret's closed-loop update method to move toward the desired
                // robot-relative angle (which will be zero after capture) and then hold the
                // world-facing angle while the robot rotates.
                turret.update(usePose);

                // Diagnostic telemetry to help determine why the turret isn't moving
                telemetry.addData("TurretDebug", turret.getDebugString());
                telemetry.addData("TurretPower", turret.getLastPower());
                telemetry.addData("TurretTargetDeg", Math.toDegrees(turret.getLastTargetAngle()));
                telemetry.addData("TurretPosDeg", turret.getRobotRelativeDeg());
                if (pinpointPose != null) {
                    telemetry.addData("PinHeadingDeg", Math.toDegrees(pinpointPose.heading.toDouble()));
                    telemetry.addData("PinRawHeadingDeg", Math.toDegrees(pinpoint.getRawHeadingRadians()));
                    telemetry.addData("PinStatus", String.valueOf(pinpoint.getDeviceStatus()));
                } else {
                    telemetry.addData("PinHeadingDeg", "n/a");
                }
                if (imu != null) {
                    double imuH = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    telemetry.addData("IMUHeadingDeg", Math.toDegrees(imuH));
                } else {
                    telemetry.addData("IMUHeadingDeg", "n/a");
                }
            } else {
                double stick = gamepad2.left_stick_x;
                double deadzone = 0.05;
                if (stick < -deadzone) {
                    turret.rotateLeft();
                } else if (stick > deadzone) {
                    turret.rotateRight();
                } else {
                    turret.stop();
                }
            }

            // update previous button state
            prevTriangle = gamepad2.triangle;

            // --- HOOD CONTROL (D-Pad) ---
//            if (gamepad2.dpad_right) {
//                hood.up(); // Preset Up
//            } else if (gamepad2.dpad_left) {
//                hood.down(); // Preset Down
//            } else if (gamepad2.dpad_up) {
//                hood.moveHood(0.005); // Fine Adjustment Up
//            } else if (gamepad2.dpad_down) {
//                hood.moveHood(-0.005); // Fine Adjustment Down
//            }
            if (gamepad2.dpad_up) {
                hood.setPosition(Constants.HOOD_UPPER_LIMIT);
            }

            if (gamepad2.dpad_right) {
                hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
            }

            if (gamepad2.dpad_down) {
                hood.setPosition(Constants.HOOD_LOWER_LIMIT);
            }

            if (gamepad2.left_bumper) {
                lift.up();
            }

            if (gamepad2.right_bumper) {
                lift.down();
            }

            drive.update(drive.getPose());

            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Velocity", drive.shooter.getVelocity());
            telemetry.addData("Target Velo", (leftTriggerVal > 0.001) ? Constants.SHOOTER_VELOCITY : 0.0);
            telemetry.addData("Hood Pos", hood.getPosition());
            telemetry.update();
        }
    }
}
