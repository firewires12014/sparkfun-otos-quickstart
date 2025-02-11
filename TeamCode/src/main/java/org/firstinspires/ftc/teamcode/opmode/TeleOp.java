package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setColorRed();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            //Drive

            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (gamepad1.right_bumper) robot.arm.grabber.setPosition(Arm.OPEN);

            // Operator

            //Intake & Color detection

            if (!scheduler.isBusy()) {
                robot.intake.manualControl(-gamepad2.left_stick_y);
            }

            if (gamepad2.touchpad) {
                if (Intake.selected_color.equalsIgnoreCase("R")) {
                    robot.intake.setColorBlue();
                    gamepad2.setLedColor(0, 0, 255, -1);
                } else {
                    robot.intake.setColorRed();
                    gamepad2.setLedColor(255, 0, 0, -1);
                }
            }
            telemetry.addData("Selected Color", Intake.selected_color);

            if (gamepad2.right_trigger > 0.1) {
                robot.intake.spin.setPower(1);
                intake.intakeDown();
            } else if (gamepad2.left_trigger > 0.1 && !scheduler.isBusy()) {
                scheduler.queueAction(robot.eject());
            } else {
                robot.intake.spin.setPower(0);
            }

            if (robot.intake.hasSample() && !robot.intake.isRightColor() && !scheduler.isBusy()) {
               // scheduler.queueAction(robot.eject());
            }

            if (robot.intake.hasSample() && robot.intake.isRightColor() && !scheduler.isBusy()) {
                scheduler.queueAction(robot.transfer());
            }

            if ((gamepad2.left_bumper) && !scheduler.isBusy()) {
                //at some point FSM that moves the arm out the first press and grabs & primes drop the second press
                robot.arm.specIntake();
            }

            if (gamepad2.cross) {
                //close claw
                robot.arm.grab();
            }

            //Outtake

            if (!scheduler.isBusy()) {
                robot.lift.manualControl(-gamepad2.right_stick_y);
            }
            if ((gamepad2.dpad_down) && !scheduler.isBusy()) {
                //specimen
                robot.outtakeSpec();
            }

            if ((gamepad2.dpad_right) && !scheduler.isBusy()) {
                //observation drop
                robot.outtakeObservation();
            }

            if ((gamepad2.dpad_left) && !scheduler.isBusy()) {
                //low bucket
                robot.outtakeLowBucket();
            }

            if ((gamepad2.dpad_up) && !scheduler.isBusy()) {
                //high bucket
                robot.outtakeBucket();
            }

            if ((gamepad2.right_bumper) && !scheduler.isBusy()) {
                //drop & return
                if (arm.wristPosition == arm.WRIST_BUCKET_PRIME) {
                    scheduler.queueAction(robot.sampleDropAndReturn());
                } else {
                    scheduler.queueAction(robot.dropAndReturn());
                }
            }
                if (gamepad2.triangle) {
                robot.lift.resetEncoder();
                robot.intake.resetEncoder();
            }
            //Hang

            if ((gamepad2.square) && !scheduler.isBusy()) {
                scheduler.queueAction(robot.hang.hangIn());
            }

            if ((gamepad2.circle) && !scheduler.isBusy()) {
                scheduler.queueAction(robot.hang.hangOut());
            }
            telemetry.update();
            scheduler.update();
            robot.update();
        }
    }}
