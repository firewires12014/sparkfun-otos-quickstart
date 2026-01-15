// filepath: c:\Code\roadrunner-decode\decode\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\TeleopRoadrunner.java
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "TeleopRoadrunner", group = "Linear OpMode")
public class TeleopRoadrunner extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // drive scaling (mapped to MecanumDrive.PARAMS)
    private double maxWheelVel;
    private double maxAngVel;

    public static int velocity = 1800; // kept for compatibility with Hardware tuning
    public static int hoodUpperLimit = 1; // servo positions (0.0 - 1.0)
    public static int hoodLowerLimit = 0;
    public static double lift1Power = -1.0;
    public static double lift2Power = 1.0;

    // Turret tunables visible in FTC Dashboard
    public static double turretTicksPerRevolution = 8192.0; // counts per revolution (encoder CPR)
    public static double turretGearRatio = 1.0; // motor-to-turret gear ratio
    public static double turretKp = 2.5; // P gain for turret servo (tweak)
    public static double turretAngleLimitDeg = 90.0; // +/- limit from initial (degrees)
    public static double manualDeadband = 0.12;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware and drive
        Hardware robot = new Hardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // read parameters for scaling
        maxWheelVel = MecanumDrive.PARAMS.maxWheelVel;
        maxAngVel = MecanumDrive.PARAMS.maxAngVel;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // lifts and encoders
        robot.liftCH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftCH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftEH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        boolean intakeOn = false;
        boolean lastA = false;

        // Turret world-stabilization state (use encoder feedback)
        double lastTime = runtime.seconds();
        double initialHeading = drive.localizer.getPose().heading.toDouble();
        double lastHeading = initialHeading;
        boolean autoTurret = true; // enabled by default
        boolean lastX = false;
        boolean lastY = false;

        // Encoder-based turret state
        int turretEncoderZero = robot.turretEncoder.getCurrentPosition();
        // turret angle at initialization (radians)
        double initialTurretAngle = 0.0; // treated as zero reference

         while (opModeIsActive()) {
            // recompute turret angle limit from dashboard-controlled degrees so updates take effect live
            double turretAngleLimit = Math.toRadians(turretAngleLimitDeg);

            // Read joysticks
            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Scale to drive's velocity units
            double vx = y * maxWheelVel;       // axial (forward)
            double vy = x * maxWheelVel;       // lateral (strafe)
            double omega = rx * maxAngVel;    // angular velocity (rad/s)

            // Build Roadrunner command (robot-frame velocities)
            PoseVelocity2d cmd = new PoseVelocity2d(new Vector2d(vx, vy), omega);

            // Send command to Roadrunner drive (it will compute motor powers)
            drive.setDrivePowers(cmd);

            // Optionally update pose estimate (keeps localizer fresh) and get timing
            PoseVelocity2d vel = drive.updatePoseEstimate();

            double now = runtime.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-3;
            lastTime = now;

            double heading = drive.localizer.getPose().heading.toDouble();
            double robotAngularVel = (heading - lastHeading) / dt;
            lastHeading = heading;

            // Intake toggle (gamepad1.a)
            if (gamepad1.a && !lastA) {
                intakeOn = !intakeOn;
            }
            lastA = gamepad1.a;
            robot.intake.setPower(intakeOn ? 1.0 : 0.0);

            // Shooter: right bumper to spin up, left bumper to stop
            if (gamepad1.right_bumper) {
                Hardware.targetVelocity = velocity;
                Hardware.shoot = true;
            } else if (gamepad1.left_bumper) {
                Hardware.shoot = false;
            }

            // transfer rollers (gamepad2 right trigger)
            double transferPower = gamepad2.right_trigger > 0.1 ? 1.0 : 0.0;
            robot.transfer1.setPower(transferPower);
            robot.transfer2.setPower(transferPower);

            // turret: encoder-based world-stabilization with manual override
            // read current turret encoder and compute angle relative to zero
            int curTicks = robot.turretEncoder.getCurrentPosition();
            double curAngle = ((double)(curTicks - turretEncoderZero) / turretTicksPerRevolution) * 2.0 * Math.PI / turretGearRatio;

            // reset/toggle buttons (debounced) for recenter/toggle auto
            if (gamepad2.x && !lastX) {
                // re-center turret reference: treat current turret angle as initial
                turretEncoderZero = robot.turretEncoder.getCurrentPosition();
                initialHeading = heading;
                initialTurretAngle = 0.0;
            }
            if (gamepad2.y && !lastY) {
                autoTurret = !autoTurret;
            }
            lastX = gamepad2.x;
            lastY = gamepad2.y;

            double manual = gamepad2.left_stick_x;
            if (Math.abs(manual) > manualDeadband) {
                // manual control - disable auto while stick is moved
                autoTurret = false;
                double power = manual;
                robot.turret.setPower(power);
            } else {
                // when manual stick is released, fall back to auto if enabled
                if (autoTurret) {
                    // desired turret angle to keep world aim: initialTurretAngle - (heading - initialHeading)
                    double desiredAngle = initialTurretAngle - (heading - initialHeading);
                    // clamp desired angle to +/- limit
                    double minAngle = -turretAngleLimit;
                    double maxAngle = turretAngleLimit;
                    if (desiredAngle > maxAngle) desiredAngle = maxAngle;
                    if (desiredAngle < minAngle) desiredAngle = minAngle;

                    // error between desired and current
                    double err = desiredAngle - curAngle;

                    // simple P controller -> power
                    double power = turretKp * err;
                    if (power > 1.0) power = 1.0;
                    if (power < -1.0) power = -1.0;

                    // If already at physical limit (approx by encoder) stop driving further out
                    if ((curAngle >= maxAngle && power > 0) || (curAngle <= minAngle && power < 0)) {
                        power = 0;
                    }

                    robot.turret.setPower(power);
                } else {
                    // if autoTurret disabled and stick released, hold current position by zeroing desired reference
                    initialTurretAngle = curAngle;
                    initialHeading = heading;
                    robot.turret.setPower(0);
                }
            }

            // hood control using dpad up/down
            if (gamepad2.dpad_up) {
                robot.hood.setPosition(hoodUpperLimit);
            } else if (gamepad2.dpad_down) {
                robot.hood.setPosition(hoodLowerLimit);
            }

            // lifts controlled by gamepad2 sticks
            robot.liftCH.setPower(gamepad2.right_stick_y * lift2Power);
            robot.liftEH.setPower(gamepad2.left_stick_y * lift1Power);

            // Let Hardware perform periodic updates (shooter PID, etc.)
            robot.update();

            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive (vx, vy, omega)", "%.2f, %.2f, %.2f", vx, vy, omega);
            telemetry.addData("TurretAuto", autoTurret);
            telemetry.addData("TurretAngleDeg", String.format("%.1f", Math.toDegrees(curAngle)));
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("Shooting", Hardware.shoot);
            telemetry.addData("TargetVel", Hardware.targetVelocity);
            telemetry.update();

            idle();
        }

        // stop motors on exit
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);

        robot.intake.setPower(0);
        robot.transfer1.setPower(0);
        robot.transfer2.setPower(0);
        robot.turret.setPower(0);
        robot.shooter.setPower(0);
    }
}
