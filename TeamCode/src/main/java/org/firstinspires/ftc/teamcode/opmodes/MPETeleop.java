package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.FireBot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.TurretLUT;

import java.util.List;

/*
 * MPE Teleop -- main driver-controlled OpMode.
 *
 *   Driver 1 (gamepad1): drives the robot, picks alliance, and resets pose.
 *   Driver 2 (gamepad2): runs intake, transfer, shooter, turret, hood, lift.
 *
 * Aiming strategy:
 *   - At init, the Limelight polls for an AprilTag to establish the starting pose.
 *   - During the match, turret aim and shooter RPM are computed from odometry (fast).
 *   - Every loop the Limelight checks for an AprilTag; if one is visible it resets
 *     the odometry pose to correct any accumulated drift.
 *
 * The @Config annotation exposes the public-static fields below to FTC Dashboard
 * so they can be tuned live without rebuilding.
 */
@Config
@TeleOp(name = "01. MPE-Teleop", group = "MPE")
public class MPETeleop extends LinearOpMode {

    // =========================================================================
    // DASHBOARD-TUNABLE CONFIG (edit live from FTC Dashboard while disabled)
    // =========================================================================

    // Joystick conditioning -- passed to FireBot.joystick_conditioning.
    public static float deadband = 0f;
    public static float offset   = 0.02f;
    public static float gain     = 0.7f;

    public static final double BLUELEDCOLOR = 0.611;
    public static final double REDLEDCOLOR = 0.29;

    // Beam-break (distance-sensor) tuning for the ball counter.
    public static final double BEAM_DEBOUNCE       = 0.25;  // seconds between counts
    public static final double BEAM_THRESHOLD_INCH = 4.0;   // ball "seen" closer than this

    // Goal positions on the field (used for auto-aim + LUT distance lookup).
    public static Vector2d RED_GOAL  = new Vector2d(-72,  72);
    public static Vector2d BLUE_GOAL = new Vector2d(-72, -72);

    // =========================================================================
    // RUNTIME STATE (changes during the match)
    // =========================================================================

    public static boolean isBlue      = true;   // current alliance color
    public static boolean autoTurret  = true;   // true = auto-aim, false = manual stick
    public static double  hoodPosition = 0;     // commanded hood servo position
    public static double  shooterRPM   = 0;     // commanded flywheel RPM

    public int ballCount = 3;                   // balls remaining (counted by beam break)

    // Beam-break debounce state.
    private double  lastBeamTime   = -1.0;
    private boolean prevBeamBroken = false;

    // Tracks the falling edge of the shoot trigger so we can refill LEDs once.
    private boolean wasShooting = false;

    private final ElapsedTime runtime = new ElapsedTime();

    public TurretLUT turretLUT;
    private Vision vision;

    // Distinguishes red from blue when looking up goal positions.
    public enum Alliance { RED, BLUE }

    @Override
    public void runOpMode() throws InterruptedException {
        // Mirror telemetry to BOTH the Driver Station and FTC Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---------------------------------------------------------------------
        // SUBSYSTEM INITIALIZATION
        // ---------------------------------------------------------------------
        Drive    drive    = new Drive(hardwareMap);
        Intake   intake   = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Turret   turret   = new Turret(hardwareMap);
        Hood     hood     = new Hood(hardwareMap);
        Shooter  shooter  = new Shooter(hardwareMap);
        Hardware robot    = new Hardware(hardwareMap);
        FireBot  fireBot  = new FireBot();   // helper for joystick conditioning
        vision = new Vision(robot.limelight, robot.turret);

        // Make sure the transfer's trigger is closed before we move balls.
        transfer.triggerClose();

        // Show 3 balls in the alliance color on the LED strip.
        updateLeds(robot, ballCount, isBlue);

        // ---------------------------------------------------------------------
        // SHOOTER LOOKUP TABLE
        // For each measured distance to the goal (inches), the LUT stores the
        // flywheel RPM and hood servo position that scored cleanly.
        // ---------------------------------------------------------------------
        turretLUT = new TurretLUT(List.of(
                new TurretLUT.Datapoint(37.7,   new TurretLUT.ShooterConfiguration(1150, 0.04)),
                new TurretLUT.Datapoint(43,     new TurretLUT.ShooterConfiguration(1150, 0.04)),
                new TurretLUT.Datapoint(59,     new TurretLUT.ShooterConfiguration(1150, 0.04)),
                new TurretLUT.Datapoint(71,     new TurretLUT.ShooterConfiguration(1165, 0.10)),
                new TurretLUT.Datapoint(80,     new TurretLUT.ShooterConfiguration(1200, 0.11)),
                new TurretLUT.Datapoint(91,     new TurretLUT.ShooterConfiguration(1300, 0.12)),
                new TurretLUT.Datapoint(100.7,  new TurretLUT.ShooterConfiguration(1400, 0.16)),
                new TurretLUT.Datapoint(133.9,  new TurretLUT.ShooterConfiguration(1600, 0.24)),
                new TurretLUT.Datapoint(141.68, new TurretLUT.ShooterConfiguration(1650, 0.22)),
                new TurretLUT.Datapoint(144.5,  new TurretLUT.ShooterConfiguration(1600, 0.23)),
                new TurretLUT.Datapoint(145,    new TurretLUT.ShooterConfiguration(1650, 0.23)),
                new TurretLUT.Datapoint(148.9,  new TurretLUT.ShooterConfiguration(1675, 0.22))
        ));

        // ---------------------------------------------------------------------
        // INIT-TIME APRIL TAG POSE FIX
        // Start the Limelight now and poll for an AprilTag to establish the
        // robot's starting position. The loop keeps refreshing until the driver
        // presses Start, so the last good fix wins.
        // ---------------------------------------------------------------------
        vision.start();
        telemetry.addData("Status", "Searching for AprilTag pose fix... select alliance then press Start");
        telemetry.update();

        boolean aprilTagDetected = false;

        while (!isStarted() && !isStopRequested()) {
            // Allow alliance selection during init so the correct trusted tag IDs are used.
            if (gamepad1.circle) {
                isBlue = false;
                gamepad1.setLedColor(255, 0, 0, -1);
                updateLeds(robot, ballCount, false);
            }
            if (gamepad1.cross) {
                isBlue = true;
                gamepad1.setLedColor(0, 0, 255, -1);
                updateLeds(robot, ballCount, true);
            }

            // Update position only once when an AprilTag is detected.
            if (!aprilTagDetected) {
                boolean gotFix = vision.tryResetPoseFromAprilTag(drive, isBlue);
                if (gotFix) {
                    aprilTagDetected = true;

                    // Blink LEDs green to indicate AprilTag detection.
                    for (int i = 0; i < 3; i++) {
                        setAllLedsGreen(robot);
                        sleep(200);
                        setAllLedsOff(robot);
                        sleep(200);
                    }

                    Pose2d initPose = drive.getPose();
                    telemetry.addData("AprilTag Fix", "YES — pose updated");
                    telemetry.addData("Init Pose X", "%.1f in", initPose.position.x);
                    telemetry.addData("Init Pose Y", "%.1f in", initPose.position.y);
                    telemetry.addData("Init Heading", "%.1f deg", Math.toDegrees(initPose.heading.toDouble()));
                } else {
                    telemetry.addData("AprilTag Fix", "searching...");
                }
            }

            telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
            telemetry.update();
            sleep(50);
        }

        // =====================================================================
        // MAIN LOOP -- runs many times per second until the OpMode is stopped.
        // =====================================================================
        runtime.reset();

        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // 1) SENSORS & ODOMETRY
            // -----------------------------------------------------------------

            // Read triggers once at the top so all sections share the same values.
            double  leftTriggerVal  = Math.max(gamepad1.left_trigger,  gamepad2.left_trigger);
            double  rightTriggerVal = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            boolean isShooting      = leftTriggerVal > 0.001;

            // Update odometry dead-reckoning from the wheel/pinpoint encoder.
            drive.update();
            Pose2d pose = drive.getPose();

            // Opportunistically correct drift: if the Limelight sees a trusted
            // AprilTag this loop, reset the odometry pose to the vision fix.
            //boolean aprilTagFix = vision.tryResetPoseFromAprilTag(drive, isBlue);
            //if (aprilTagFix) {
            //    pose = drive.getPose(); // re-read the corrected pose
            //}

            // Compute once here; reused by auto-aim and telemetry.
            double dist = calculateGoalDistance(pose, isBlue ? Alliance.BLUE : Alliance.RED);

            // Count balls passing the intake distance sensor (rising-edge + debounce).
            // Only check while the driver is actively intaking (right trigger).
            if (rightTriggerVal > 0) {
                double sensorInches = robot.distanceSensor.getDistance(DistanceUnit.INCH);
                boolean beamBroken  = sensorInches > 0 && sensorInches < BEAM_THRESHOLD_INCH;

                if (beamBroken && !prevBeamBroken
                        && (runtime.seconds() - lastBeamTime) > BEAM_DEBOUNCE) {
                    lastBeamTime = runtime.seconds();
                    if (ballCount > 0) {
                        ballCount -= 1;
                        updateLeds(robot, ballCount, isBlue);
                    }
                }
                prevBeamBroken = beamBroken;
            } else {
                // Trigger released: reset edge-state so the next intake has a clean rising edge.
                prevBeamBroken = false;
            }

            // -----------------------------------------------------------------
            // 2) GAMEPAD 1 -- DRIVE STICKS
            // -----------------------------------------------------------------
            double axial   = fireBot.joystick_conditioning(-gamepad1.left_stick_y, deadband, offset, gain);
            double lateral = fireBot.joystick_conditioning( gamepad1.left_stick_x, deadband, offset, gain);
            double yaw     = fireBot.joystick_conditioning( gamepad1.right_stick_x, deadband, offset, gain);
            drive.drive(axial, lateral, yaw);

            // -----------------------------------------------------------------
            // 3) BOTH GAMEPADS -- GAME PIECE CONTROLS
            // -----------------------------------------------------------------

            // --- INTAKE / TRANSFER STATE MACHINE ---
            // Priority order: cross > circle > right-trigger > idle.
            // Holding the LEFT trigger (shoot) falls through to the final else.
            if (gamepad2.cross && leftTriggerVal == 0) {
                // Cross: intake AND run transfer (move ball into shooter chamber).
                intake.in();
                transfer.run();
            } else if (gamepad2.circle && leftTriggerVal == 0) {
                // Circle: reverse everything (clear a jam).
                intake.out();
                transfer.reverse();
                shooter.reverse();
            } else if (rightTriggerVal > 0 && leftTriggerVal == 0) {
                // Right trigger only: pull balls in but keep the trigger closed
                // so a ball stays staged. The beam-break above counts here.
                intake.in();
                transfer.stop();
                transfer.triggerClose();
            } else if (rightTriggerVal == 0 && leftTriggerVal == 0) {
                // Idle: stop everything and zero shooter RPM.
                intake.stop();
                transfer.stop();
                shooterRPM = 0;
            } else {
                // Left trigger held: open the trigger to feed the flywheel.
                transfer.triggerOpen();
            }

            // --- LED REFILL ON SHOT-RELEASE ---
            if (!isShooting && wasShooting) {
                setAllLedsOn(robot, isBlue);
                ballCount = 3;
            }
            wasShooting = isShooting;

            // Safety stop: if no input is active, force the intake to idle.
            if (leftTriggerVal <= 0.1 && !gamepad2.cross && !gamepad2.circle && rightTriggerVal == 0) {
                intake.stop();
            }

            // --- TURRET MODE TOGGLE ---
            if (gamepad2.triangle) autoTurret = false;   // manual
            if (gamepad2.square)   autoTurret = true;    // auto

            // --- TURRET / SHOOTER CONTROL ---
            double targetAngle = 0; // populated in auto mode; used in telemetry
            if (!autoTurret) {
                // Manual: left stick X steers the turret; left trigger spins the flywheel.
                if (leftTriggerVal > 0.1) {
                    shooterRPM = Constants.SHOOTER_VELOCITY;
                }

                double stick    = gamepad2.left_stick_x;
                double deadzone = 0.05;
                if (Math.abs(stick) > deadzone) {
                    turret.increment(stick);
                }
            } else {
                // Auto: compute turret angle from odometry, look up RPM + hood from LUT.
                // Pose was already corrected by AprilTag above (if a tag was visible).
                Vector2d goalPos = isBlue ? BLUE_GOAL : RED_GOAL;
                targetAngle      = findTargetAngle(new Pose2d(goalPos, 0), pose);
                turret.setAngle(targetAngle);

                TurretLUT.ShooterConfiguration config = turretLUT.calculate(dist);
                shooterRPM   = config.getFlywheelRPM();
                hoodPosition = config.getHoodServoPosition();
            }

            // Drive the flywheel: spins up when trigger is held, cuts power when released.
            shooter.update(isShooting, (int) shooterRPM);

            // --- HOOD CONTROL ---
            if (!autoTurret) {
                if (gamepad2.dpad_up)    hood.setPosition(Constants.HOOD_UPPER_LIMIT);
                if (gamepad2.dpad_right) hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
                if (gamepad2.dpad_down)  hood.setPosition(Constants.HOOD_LOWER_LIMIT);
            } else {
                hood.setPosition(hoodPosition);
            }

            // --- LIFT (endgame climb) ---
            if (gamepad2.left_bumper) {
                robot.lift1.setPosition(Constants.LIFT1_UP);
                robot.lift2.setPosition(Constants.LIFT2_UP);
            }
            if (gamepad2.right_bumper) {
                robot.lift1.setPosition(Constants.LIFT1_DOWN);
                robot.lift2.setPosition(Constants.LIFT2_DOWN);
            }

            // -----------------------------------------------------------------
            // 4) GAMEPAD 1 -- ALLIANCE / POSE RESET
            // Runs LAST so its LED writes override the ball-count LEDs above.
            // -----------------------------------------------------------------

            if (gamepad1.circle) {
                gamepad1.setLedColor(255, 0, 0, -1);
                isBlue = false;
                robot.led1.setPosition(REDLEDCOLOR);
                robot.led2.setPosition(REDLEDCOLOR);
                robot.led3.setPosition(REDLEDCOLOR);
            }

            if (gamepad1.cross) {
                gamepad1.setLedColor(0, 0, 255, -1);
                isBlue = true;
                robot.led1.setPosition(BLUELEDCOLOR);
                robot.led2.setPosition(BLUELEDCOLOR);
                robot.led3.setPosition(BLUELEDCOLOR);
            }

            if (gamepad1.triangle) {
                drive.setPose(new Pose2d(-63, 0, Math.toRadians(90)));
            }

            // -----------------------------------------------------------------
            // 5) TELEMETRY
            // -----------------------------------------------------------------
            Double tx = vision.getLatestTx();

            telemetry.addData("",  "[%s | %s]  %.0fs",
                    isBlue ? "BLU" : "RED", autoTurret ? "AUTO" : "MAN", runtime.seconds());
            telemetry.addData("Balls",  ballCount + " / 3");
            telemetry.addData("RPM",   "%.0f → %.0f",
                    robot.shooter.getVelocity(), isShooting ? shooterRPM : 0.0);
            telemetry.addData("Hood",  "%.3f", hood.getPosition());
            telemetry.addData("Dist",  "%.1f in", dist);
            if (autoTurret) {
                telemetry.addData("Turret", "%.1f°", targetAngle);
            }
            telemetry.addData("LL",    tx != null
                    ? String.format("YES  tx=%.1f°", tx)
                    : "no tag");
            telemetry.addData("Pose",  "(%.1f, %.1f)  %.1f°",
                    pose.position.x, pose.position.y,
                    Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }

        vision.stop();
    }

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    /**
     * Computes the turret angle (degrees) needed to point from the robot's current
     * pose toward the target pose, expressed in the robot's heading frame.
     */
    public double findTargetAngle(Pose2d target, Pose2d current) {
        double dx = target.position.x - current.position.x;
        double dy = target.position.y - current.position.y;
        double relativeAngle = Math.atan2(dy, dx);
        double turretAngle   = AngleUnit.normalizeRadians(relativeAngle - current.heading.toDouble());
        return Math.toDegrees(-turretAngle);
    }

    /** Straight-line distance from the robot to the given alliance's goal. */
    public static double calculateGoalDistance(Pose2d currentPose, Alliance alliance) {
        Vector2d goalPos = (alliance == Alliance.RED) ? RED_GOAL : BLUE_GOAL;
        double dx = currentPose.position.x - goalPos.x;
        double dy = currentPose.position.y - goalPos.y;
        return Math.hypot(dx, dy);
    }

    /** Light up one LED per remaining ball (alliance-colored). 0 balls = all off. */
    private void updateLeds(Hardware robot, int ballCount, boolean isBlue) {
        double onPos  = isBlue ? 0.611 : 0.29;
        double offPos = 0;
        robot.led1.setPosition(ballCount >= 1 ? onPos : offPos);
        robot.led2.setPosition(ballCount >= 2 ? onPos : offPos);
        robot.led3.setPosition(ballCount >= 3 ? onPos : offPos);
    }

    /** Force all three LEDs on (used to indicate "magazine refilled"). */
    private void setAllLedsOn(Hardware robot, boolean isBlue) {
        double onPos = isBlue ? 0.611 : 0.29;
        robot.led1.setPosition(onPos);
        robot.led2.setPosition(onPos);
        robot.led3.setPosition(onPos);
    }

    /** Blink LEDs green to indicate AprilTag detection. */
    private void setAllLedsGreen(Hardware robot) {
        double greenPos = 0.5; // Adjust this value for green LED position.
        robot.led1.setPosition(greenPos);
        robot.led2.setPosition(greenPos);
        robot.led3.setPosition(greenPos);
    }

    /** Turn off all LEDs. */
    private void setAllLedsOff(Hardware robot) {
        double offPos = 0;
        robot.led1.setPosition(offPos);
        robot.led2.setPosition(offPos);
        robot.led3.setPosition(offPos);
    }
}
