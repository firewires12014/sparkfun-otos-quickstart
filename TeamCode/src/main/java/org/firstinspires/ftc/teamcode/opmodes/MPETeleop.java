package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Beam-break (distance-sensor) tuning for the ball counter.
    public static final double BEAM_DEBOUNCE      = 0.25;  // seconds between counts
    public static final double BEAM_THRESHOLD_INCH = 4.0;  // ball "seen" closer than this

    // Goal positions on the field (used for auto-aim + LUT distance lookup).
    public static Vector2d RED_GOAL  = new Vector2d(-72,  72);
    public static Vector2d BLUE_GOAL = new Vector2d(-72, -72);

    // =========================================================================
    // RUNTIME STATE (changes during the match)
    // =========================================================================

    public static boolean isBlue       = true;   // current alliance color
    public static boolean autoTurret   = true;   // true = auto-aim, false = manual stick
    public static double  hoodPosition = 0;      // commanded hood servo position
    public static double  shooterRPM   = 0;      // commanded flywheel RPM

    public int ballCount = 3;                    // balls remaining (counted by beam break)

    // Beam-break debounce state.
    private double  lastBeamTime    = -1.0;
    private boolean prevBeamBroken  = false;

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
        Turret   turret   = new Turret(hardwareMap);   // kept so the motor inits; rotation will be Limelight-driven
        Hood     hood     = new Hood(hardwareMap);
        Shooter  shooter  = new Shooter(hardwareMap);
        Hardware robot    = new Hardware(hardwareMap);
        FireBot  FireBot  = new FireBot();   // helper for joystick conditioning
        vision = new Vision(robot.limelight, robot.turret);

        // Make sure the transfer's trigger is closed before we move balls.
        transfer.triggerClose();

        // Show 3 balls in the alliance color on the LED strip.
        updateLeds(robot, ballCount, isBlue);

        // ---------------------------------------------------------------------
        // SHOOTER LOOKUP TABLE
        // For each measured distance to the goal (inches), the LUT stores the
        // flywheel RPM and hood servo position that scored cleanly. The Turret
        // subsystem interpolates between these points at runtime.
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
                new TurretLUT.Datapoint(144.5,  new TurretLUT.ShooterConfiguration(1600, 0.23)),
                new TurretLUT.Datapoint(145,    new TurretLUT.ShooterConfiguration(1650, 0.23)),
                new TurretLUT.Datapoint(148.9,  new TurretLUT.ShooterConfiguration(1675, 0.22)),
                new TurretLUT.Datapoint(141.68, new TurretLUT.ShooterConfiguration(1650, 0.22))
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        vision.start();

        // =====================================================================
        // MAIN LOOP -- runs many times per second until the OpMode is stopped.
        // =====================================================================
        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // 1) SENSORS & ODOMETRY
            // -----------------------------------------------------------------

            // Count balls passing the intake distance sensor (rising-edge + debounce).
            // Only check while the driver is actively intaking (right trigger).
            if (gamepad2.right_trigger > 0) {
                double sensorInches = robot.distanceSensor.getDistance(DistanceUnit.INCH);
                boolean beamBroken  = sensorInches > 0 && sensorInches < BEAM_THRESHOLD_INCH;

                // Trigger only the moment the beam goes from clear -> broken,
                // and only if BEAM_DEBOUNCE seconds have passed since the last count.
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
                // Trigger released: reset beam-break state so next intake has a clean rising-edge.
                // This prevents stale edge-state from blocking the next ball count.
                prevBeamBroken = false;
            }

            // Update odometry and read the latest pose for turret/aim math.
            drive.update();
            Pose2d pose = drive.getPose();

            // -----------------------------------------------------------------
            // 2) GAMEPAD 1 -- DRIVE STICKS
            // -----------------------------------------------------------------
            // Joystick values are smoothed/deadbanded by FireBot.joystick_conditioning.
            double axial   = FireBot.joystick_conditioning(-gamepad1.left_stick_y, deadband, offset, gain);
            double lateral = FireBot.joystick_conditioning( gamepad1.left_stick_x, deadband, offset, gain);
            double yaw     = FireBot.joystick_conditioning( gamepad1.right_stick_x, deadband, offset, gain);
            drive.drive(axial, lateral, yaw);

            // -----------------------------------------------------------------
            // 3) GAMEPAD 2 -- GAME PIECE CONTROLS
            // -----------------------------------------------------------------

            // --- READ TRIGGER STATE ---
            double  leftTriggerVal  = gamepad2.left_trigger;   // shoot
            double  rightTriggerVal = gamepad2.right_trigger;  // intake
            boolean isShooting      = leftTriggerVal > 0.001;

            // NOTE: shooter.setVelocity() is deferred until AFTER the LUT lookup in auto mode.
            // This ensures the shooter uses the current loop's RPM, not the previous loop's value.
            // See below in the auto-turret block (line ~258).

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
                telemetry.addLine("Trigger closed");
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
            // When the driver lets off the shoot trigger, assume we just emptied
            // the magazine and reset to 3 balls (matches our reload workflow).
            // The all-on LED shows momentarily as a visual confirmation of the refill.
            if (!isShooting && wasShooting) {
                setAllLedsOn(robot, isBlue);
                ballCount = 3;
            }
            wasShooting = isShooting;
            
            // NOTE: Ball count varies (1-3 balls) depending on intake timing and beam-break sensor.
            // The LED display accurately reflects actual ball count; inconsistency is expected.

            // Safety stop: if no input is active, force the intake to idle.
            if (leftTriggerVal <= 0.1 && !gamepad2.cross && !gamepad2.circle && rightTriggerVal == 0) {
                intake.stop();
            }

            // --- TURRET MODE TOGGLE ---
            if (gamepad2.triangle) autoTurret = false;   // manual mode
            if (gamepad2.square)   autoTurret = true;    // auto mode (LUT-driven shooter)

            // --- TURRET CONTROL ---
            // Turret rotation is driven by the Limelight3A.

            if (!autoTurret) {
                // Manual: hold left trigger to spin the flywheel at SHOOTER_VELOCITY.
                if (leftTriggerVal > 0.1) {
                    shooterRPM = Constants.SHOOTER_VELOCITY;
                }
                // Push shooter RPM every loop (whether trigger is pressed or released).
                shooter.setVelocity(shooterRPM);
            } else {
                // Auto: aim via Limelight, look up RPM + hood from the LUT.
                vision.updatePoseAndAimFromLimelight(drive, isBlue);
                // CRITICAL: Re-read pose AFTER vision update to get vision-corrected position.
                // Otherwise, distance calculation uses stale pre-update odometry.
                pose = drive.getPose();
                Double tx = vision.getLatestTx();
                if (tx != null && turret != null) {
                    telemetry.addData("LL tx", tx);
                    telemetry.addData("Turret Servo", vision.getTurretServoPosition());
                } else {
                    telemetry.addData("LL tx", "no target");
                    telemetry.addData("Turret Servo", vision.getTurretServoPosition());
                }

                double dist = calculateGoalDistance(pose, isBlue ? Alliance.BLUE : Alliance.RED);
                TurretLUT.ShooterConfiguration config = turretLUT.calculate(dist);
                shooterRPM   = config.getFlywheelRPM();
                hoodPosition = config.getHoodServoPosition();
                // NOW push the shooter RPM after the LUT has set it for this loop.
                shooter.setVelocity(shooterRPM);
            }

            // --- HOOD CONTROL ---
            if (!autoTurret) {
                // Manual: D-pad picks one of three preset hood positions.
                if (gamepad2.dpad_up)    hood.setPosition(Constants.HOOD_UPPER_LIMIT);
                if (gamepad2.dpad_right) hood.setPosition(Constants.HOOD_MIDDLE_LIMIT);
                if (gamepad2.dpad_down)  hood.setPosition(Constants.HOOD_LOWER_LIMIT);
            } else {
                // Auto: use whatever the LUT gave us.
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

            // Circle = RED alliance.
            // All-on momentary indicator to confirm alliance selection to driver.
            // After this, LEDs revert to ball-count display on next beam-break or shot-release.
            if (gamepad1.circle) {
                gamepad1.setLedColor(255, 0, 0, -1);
                isBlue = false;
                robot.led1.setPosition(0.29);
                robot.led2.setPosition(0.29);
                robot.led3.setPosition(0.29);
            }

            // Cross = BLUE alliance.
            // All-on momentary indicator to confirm alliance selection to driver.
            // After this, LEDs revert to ball-count display on next beam-break or shot-release.
            if (gamepad1.cross) {
                gamepad1.setLedColor(0, 0, 255, -1);
                isBlue = true;
                robot.led1.setPosition(0.611);
                robot.led2.setPosition(0.611);
                robot.led3.setPosition(0.611);
            }

            // -----------------------------------------------------------------
            // 5) APPLY FINAL COMMANDED VALUES
            // -----------------------------------------------------------------
            shooter.update(isShooting, (int) shooterRPM);

            // -----------------------------------------------------------------
            // 6) TELEMETRY
            // -----------------------------------------------------------------
            telemetry.addData("Status",          "Run Time: " + runtime);
            telemetry.addData("Is Shooting",     isShooting);
            telemetry.addData("Velocity",        robot.shooter.getVelocity());
            telemetry.addData("Target Velo",     (leftTriggerVal > 0.001) ? shooterRPM : 0.0);
            telemetry.addData("Hood Pos",        hood.getPosition());
            telemetry.addData("Ball Count",      ballCount);
            telemetry.addData("distanceToGoal",  calculateGoalDistance(pose, isBlue ? Alliance.BLUE : Alliance.RED));
            telemetry.addData("position",        pose.position);
            telemetry.update();
        }

        vision.stop();
    }

    // =========================================================================
    // HELPER METHODS
    // =========================================================================

    /** Straight-line distance from the robot to the given alliance's goal. */
    public static double calculateGoalDistance(Pose2d currentPose, Alliance alliance) {
        Vector2d goalPos = (alliance == Alliance.RED) ? RED_GOAL : BLUE_GOAL;
        double dx = currentPose.position.x - goalPos.x;
        double dy = currentPose.position.y - goalPos.y;
        return Math.hypot(dx, dy);
    }

    /** Light up one LED per remaining ball (alliance-colored). 0 balls = all off. */
    private void updateLeds(Hardware robot, int ballCount, boolean isBlue) {
        double onPos  = isBlue ? 0.611 : 0.29;   // servo positions for blue / red
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
}
