package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Hardware.*;


@Config
@TeleOp(name="Teleop", group="Linear OpMode")
public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    public static int velocity = 1900;
    // ramp rate in RPM per second (tunable via dashboard)
    public static double velocityRampRate = 1500.0;
    public static float hoodUpperLimit = .7f;
    public static float hoodLowerLimit = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware robot = new Hardware(hardwareMap);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // target velocity that will be ramped
        double targetVel = 0.0;
        // timer to compute loop delta time for smooth ramping
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            robot.frontLeft.setPower(frontLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backLeft.setPower(backLeftPower);
            robot.backRight.setPower(backRightPower);

            if (gamepad2.cross) {
                robot.intake.setPower(1);
                robot.transfer.setPower(1);
            } else if (gamepad2.circle) {
                robot.intake.setPower(-1);
                robot.transfer.setPower(-1);
                robot.shooter.setPower(-.5);
            } else if (gamepad2.right_trigger > 0) {
                robot.intake.setPower(1);
            } else {
                robot.transfer.setPower(-.25);
                robot.intake.setPower(0);
            }



            // Enable shooter
//            robot.shoot = gamepad2.left_trigger > 0;

            // Smooth ramp for shooter velocity:
            double triggerVal = gamepad2.left_trigger; // 0.0 .. 1.0
            if (triggerVal > 0.001) {
                // increase target velocity; scale ramp by how far trigger is pressed
                targetVel += velocityRampRate * dt * triggerVal;
                if (targetVel > velocity) targetVel = velocity;
            } else {
                // decay toward zero when trigger released
                targetVel -= velocityRampRate * dt;
                if (targetVel < 0) targetVel = 0;
            }

            // apply target velocity to shooter
            robot.shooter.setVelocity(targetVel);

            if (gamepad2.left_bumper) {
                robot.turret.setPower(1);
            } else if (gamepad2.right_bumper) {
                robot.turret.setPower(-1);
            } else {
                robot.turret.setPower(0);
            }

            if (gamepad2.dpad_up) {
                robot.hood.setPosition(hoodUpperLimit);
            }

            if (gamepad2.dpad_down) {
                robot.hood.setPosition(hoodLowerLimit);
            }

            robot.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocity", robot.shooter.getVelocity());
            telemetry.addData("Target Velo", targetVel);
            telemetry.addData("Zero", 0);
            telemetry.update();
        }
    }
}
