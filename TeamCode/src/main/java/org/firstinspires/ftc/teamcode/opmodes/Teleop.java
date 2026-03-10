package org.firstinspires.ftc.teamcode.opmodes;

import static com.acmerobotics.roadrunner.Math.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.FireBot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TurretLUT;

import java.util.List;

@Config
@TeleOp(name = "Teleop", group = "Linear OpMode")
public class Teleop extends LinearOpMode {
    private boolean wasShooting = false; // track previous shooting state
    private double lastBeamTime = -1.0;
    public static final double BEAM_DEBOUNCE = 0.25; // seconds
    public static final double BEAM_THRESHOLD_INCH = 4.0; // tune as needed
    private boolean prevBeamBroken = false; // rising-edge state
    public int ballCount = 3;
    private final ElapsedTime runtime = new ElapsedTime();

    public static double targetX = -67;
    public static double targetY = 67;
    public static boolean isBlue = true;
    public static double hoodPosition = 0;
    public static double shooterRPM = 0;

    public static double testingHood = 0;
    public static double testingVelocity = 0;

    public static boolean autoTurret = true;

    public static Vector2d RED_GOAL = new Vector2d(-58.3727, 55.6425);
    public static Vector2d BLUE_GOAL = new Vector2d(-58.3727, -55.6425);

    public static float deadband = 0f;
    public static float offset = .02f;
    public static float gain = .7f;

    public TurretLUT turretLUT;

    public enum Alliance {
        RED, BLUE
    }

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
        Hardware robot = new Hardware(hardwareMap);
        FireBot FireBot = new FireBot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Ensure subsystems are in starting state
        transfer.triggerClose();

        // Initialize LEDs to reflect initial ball count
        updateLeds(robot, ballCount, isBlue);

        turretLUT = new TurretLUT(List.of(
                new TurretLUT.Datapoint(37.7, new TurretLUT.ShooterConfiguration(1100, 0)),
                new TurretLUT.Datapoint(43, new TurretLUT.ShooterConfiguration(1100, 0)),
                new TurretLUT.Datapoint(59, new TurretLUT.ShooterConfiguration(1100, 0)),
                new TurretLUT.Datapoint(71, new TurretLUT.ShooterConfiguration(1150, 0.06)),
                new TurretLUT.Datapoint(80, new TurretLUT.ShooterConfiguration(1150, 0.06)),
                new TurretLUT.Datapoint(91, new TurretLUT.ShooterConfiguration(1250, 0.07)),
                new TurretLUT.Datapoint(100.7, new TurretLUT.ShooterConfiguration(1350, 0.12)),
                new TurretLUT.Datapoint(133.9, new TurretLUT.ShooterConfiguration(1550, 0.2)),
                new TurretLUT.Datapoint(144.5, new TurretLUT.ShooterConfiguration(1550, 0.19)),
                new TurretLUT.Datapoint(145, new TurretLUT.ShooterConfiguration(1600, 0.19)),
                new TurretLUT.Datapoint(148.9, new TurretLUT.ShooterConfiguration(1625, 0.18)),
                new TurretLUT.Datapoint(141.68, new TurretLUT.ShooterConfiguration(1600, 0.18))
                ));



        waitForStart();
        runtime.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {

            // Only run when intake is running
            if (gamepad2.right_trigger > 0) {
                double sensorInches = robot.distanceSensor.getDistance(DistanceUnit.INCH);
                boolean beamBroken = sensorInches > 0 && sensorInches < BEAM_THRESHOLD_INCH;

                // Rising-edge + debounce: triggers once when beam goes from clear -> broken
                if (beamBroken && !prevBeamBroken && (runtime.seconds() - lastBeamTime) > BEAM_DEBOUNCE) {
                    lastBeamTime = runtime.seconds();
                    if (ballCount > 0) {
                        ballCount -= 1;
                        updateLeds(robot, ballCount, isBlue);
                    }
                }
                prevBeamBroken = beamBroken;
            }
            loopTimer.reset();

            drive.update();
            Pose2d pose = drive.getPose();

            // =========================================================================
            // GAMEPAD 1 CONTROLS
            // =========================================================================

            // --- DRIVE CONTROL ---
            double axial = FireBot.joystick_conditioning(-gamepad1.left_stick_y, deadband, offset, gain);
            double lateral = FireBot.joystick_conditioning(gamepad1.left_stick_x, deadband, offset, gain);
            double yaw = FireBot.joystick_conditioning(gamepad1.right_stick_x, deadband, offset, gain);

            drive.drive(axial, lateral, yaw);

            // =========================================================================
            // GAMEPAD 2 CONTROLS
            // =========================================================================

            // --- SHOOTER & INTAKE CONTROL ---

            // Shooter Trigger Logic
            double leftTriggerVal = gamepad2.left_trigger;
            double rightTriggerVal = gamepad2.right_trigger;
            boolean isShooting = leftTriggerVal > 0.001;

            //shooter.setVelocity(testingVelocity);

            if (gamepad2.cross && leftTriggerVal == 0) {
                // Intake Logic: Intake Only
                intake.in();
                transfer.run();
                shooter.update(isShooting, (int) shooterRPM);
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
                shooter.update(isShooting, (int) shooterRPM);
            } else if (rightTriggerVal == 0 && leftTriggerVal == 0) {
                // Idle State
                intake.stop();
                transfer.stop();
                shooterRPM  = 0;
                shooter.update(isShooting, (int) shooterRPM);
            } else {
                shooter.update(isShooting, (int) shooterRPM);
            }

            if (!isShooting && wasShooting) {
                setAllLedsOn(robot, isBlue);
                ballCount = 3;
            }
            wasShooting = isShooting;


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
            Vector2d goalPos = isBlue ? BLUE_GOAL : RED_GOAL;
            double targetAngle = findTargetAngle(new Pose2d(goalPos, 0), pose);
            if (!autoTurret) {
                if (leftTriggerVal > 0.1) {
                    shooterRPM = Constants.SHOOTER_VELOCITY;
                    shooter.update(isShooting, (int) shooterRPM);
                } else {
                    if (shooterRPM != 0) {
                        shooter.update(isShooting, (int) shooterRPM);
                    } else {
                        shooterRPM = 0;
                    }
                }
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
                    adjustment = -2;
                else adjustment = -5;

                turret.setAngle(targetAngle + adjustment);

                double dist = calculateGoalDistance(pose, (isBlue) ? Alliance.BLUE : Alliance.RED);
                TurretLUT.ShooterConfiguration config = turretLUT.calculate(dist);

                shooterRPM = config.getFlywheelRPM();
                hoodPosition = config.getHoodServoPosition();
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
//                hood.setPosition(hood.lerp(distance));
                 hood.setPosition(hoodPosition);
                //hood.setPosition(testingHood);
            }

            if (gamepad2.left_bumper) {
                robot.lift1.setPosition(Constants.LIFT1_UP);
                robot.lift2.setPosition(Constants.LIFT2_UP);
            }
            if (gamepad2.right_bumper) {
                robot.lift1.setPosition(Constants.LIFT1_DOWN);
                robot.lift2.setPosition(Constants.LIFT2_DOWN);
            }

            if (gamepad1.triangle) {
                if (isBlue) {
                    robot.led1.setPosition(0.611);
                    robot.led2.setPosition(0.611);
                    robot.led3.setPosition(0.611);
                    gamepad1.setLedColor(0, 0, 255, -1);
                    drive.setPose(new Pose2d(-63, 0, Math.toRadians(90)));
                }
                else {
                    robot.led1.setPosition(0.29);
                    robot.led2.setPosition(0.29);
                    robot.led3.setPosition(0.29);
                    gamepad1.setLedColor(255, 0, 0, -1);
                    drive.setPose(new Pose2d(-63, 0, Math.toRadians(90)));
                }

            }

            if (gamepad1.circle) {
                gamepad1.setLedColor(255, 0, 0, -1);
                isBlue = false;
                // update LEDs to reflect alliance change without changing ball count
                updateLeds(robot, ballCount, isBlue);
            }

            if (gamepad1.cross) {
                gamepad1.setLedColor(0, 0, 255, -1);
                isBlue = true;
                // update LEDs to reflect alliance change without changing ball count
                updateLeds(robot, ballCount, isBlue);
            }




            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Velocity", robot.shooter.getVelocity());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Target Velo", (leftTriggerVal > 0.001) ? shooterRPM : 0.0);
            telemetry.addData("Hood Pos", hood.getPosition());
            telemetry.addData("Turret Position", robot.turret.getPosition());
//            telemetry.addData("Distance Sensor (in)", sensorInches);
//            telemetry.addData("Beam Broken", beamBroken);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("distanceToGoal", calculateGoalDistance(pose, (isBlue) ? Alliance.BLUE : Alliance.RED));
            telemetry.addData("position", pose.position);
            telemetry.update();

        }
    }
    public double findTargetAngle (Pose2d target, Pose2d current) {
        double dx = target.position.x - current.position.x;
        double dy = target.position.y - current.position.y;
//        double dx = target.position.x;
//        double dy = target.position.y;
        double relativeAngle = Math.atan2(dy, dx);
        double turretAngle = AngleUnit.normalizeRadians(relativeAngle - current.heading.toDouble());

        telemetry.addData("Relative Angle (rad)", relativeAngle);
        telemetry.addData("Current Heading (rad)", current.heading);
        telemetry.addData("Turret Angle (rad)", turretAngle);
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);

        return Math.toDegrees(-turretAngle);
    }

    public static double calculateGoalDistance(Pose2d currentPose, Alliance alliance) {
        Vector2d goalPos = (alliance == Alliance.RED) ? RED_GOAL : BLUE_GOAL;
        double dx = currentPose.position.x - goalPos.x;
        double dy = currentPose.position.y - goalPos.y;
        return Math.hypot(dx, dy);
    }

    /**
     * Get shooter RPM based on current position and alliance
     */
    public static double getShooterRPM(Pose2d currentPose, Alliance alliance) {
        double dist = calculateGoalDistance(currentPose, alliance);
        return clamp(0.0019379771 * Math.pow(dist, 2) + 5.3780724 * dist + 848.22413, Constants.MIN_RPM, Constants.MAX_RPM);
    }

    /**
     * Get hood position based on current position and alliance
     */
    public static double getHoodPosition(Pose2d currentPose, Alliance alliance) {
        double dist = calculateGoalDistance(currentPose, alliance);
        if (dist < 20) {
            return Constants.MIN_HOOD; // Minimum hood position for close shots
        } else {
            return clamp(-0.0000275920 * Math.pow(dist, 2) + 0.0054981 * dist - 0.02812, Constants.MIN_HOOD, Constants.MAX_HOOD);
        }
    }

    // Helper to update LEDs based on ball count and alliance color.
    private void updateLeds(Hardware robot, int ballCount, boolean isBlue) {
        double onPos = isBlue ? 0.611 : 0.29; // positions used elsewhere for blue/red
        double offPos = 0; // position representing LED off

        // Map led1..led3 to ball slots: led1 -> first ball, led2 -> second, led3 -> third
        robot.led1.setPosition(ballCount >= 1 ? onPos : offPos);
        robot.led2.setPosition(ballCount >= 2 ? onPos : offPos);
        robot.led3.setPosition(ballCount >= 3 ? onPos : offPos);
    }

    // Helper to turn all LEDs on (used after shooting)
    private void setAllLedsOn(Hardware robot, boolean isBlue) {
        double onPos = isBlue ? 0.611 : 0.29;
        robot.led1.setPosition(onPos);
        robot.led2.setPosition(onPos);
        robot.led3.setPosition(onPos);
    }
 }
